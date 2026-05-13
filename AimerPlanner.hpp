#pragma once

/**
 * @file AimerPlanner.hpp
 * @brief TinyMPC 和直接云台计划生成。
 */

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "AimerTargetModel.hpp"
#include "logger.hpp"
#include "tinympc/tiny_api.hpp"

namespace AimerDetail
{
/// 规划器采样周期，单位 s。
inline constexpr double PLAN_DEFAULT_DT_S = 0.01;
/// 从规划 horizon 中取出的命令点索引。
inline constexpr int PLAN_HALF_HORIZON = 50;
/// TinyMPC 总 horizon 长度。
inline constexpr int PLAN_HORIZON = PLAN_HALF_HORIZON * 2;
/// MPC 开火误差门控使用的额外偏移。
inline constexpr int PLAN_SHOOT_OFFSET = 2;

/// 参考轨迹行含义：yaw 偏移、yaw 速度、pitch、pitch 速度。
using PlanTrajectory = Eigen::Matrix<double, 4, PLAN_HORIZON>;

/**
 * @brief 构建 TinyMPC 使用的 yaw/pitch 参考轨迹。
 * @param cfg Aimer 运行时配置。
 * @param target_msg 当前 tracker 目标。
 * @param delay_time 弹丸飞行前的固定预测延迟，单位 s。
 * @param bullet_speed 弹速，单位 m/s。
 * @param initial_lock_id 参考轨迹的初始锁定面索引。
 * @param trajectory 输出参考轨迹。
 * @param yaw0 输出 yaw 偏置，用于让轨迹数值保持局部。
 * @return 所有参考采样都能计算时返回 true。
 */
inline bool BuildReferenceTrajectory(const AimerConfig& cfg,
                                     const ArmorTrackerTarget& target_msg,
                                     double delay_time, double bullet_speed,
                                     int initial_lock_id,
                                     PlanTrajectory& trajectory, double& yaw0)
{
  PredictedTarget center_target{target_msg};
  center_target.Predict(delay_time);
  int reference_lock_id =
      target_msg.face_switch_observed ? initial_lock_id : target_msg.tracked_face_index;

  const auto rough_aim =
      ComputeAimCommand(cfg, center_target, bullet_speed, reference_lock_id);
  if (!rough_aim.valid)
  {
    return false;
  }

  center_target.Predict(rough_aim.fly_time);
  const auto center_aim =
      ComputeAimCommand(cfg, center_target, bullet_speed, reference_lock_id);
  if (!center_aim.valid)
  {
    return false;
  }
  yaw0 = center_aim.yaw_pitch.x();

  PredictedTarget moving_target = center_target;
  moving_target.Predict(-PLAN_DEFAULT_DT_S * (PLAN_HALF_HORIZON + 1));
  auto yaw_pitch_last =
      ComputeAimCommand(cfg, moving_target, bullet_speed, reference_lock_id);
  if (!yaw_pitch_last.valid)
  {
    return false;
  }

  moving_target.Predict(PLAN_DEFAULT_DT_S);
  auto yaw_pitch =
      ComputeAimCommand(cfg, moving_target, bullet_speed, reference_lock_id);
  if (!yaw_pitch.valid)
  {
    return false;
  }

  for (int index = 0; index < PLAN_HORIZON; ++index)
  {
    moving_target.Predict(PLAN_DEFAULT_DT_S);
    auto yaw_pitch_next =
        ComputeAimCommand(cfg, moving_target, bullet_speed, reference_lock_id);
    if (!yaw_pitch_next.valid)
    {
      return false;
    }

    const double yaw_vel =
        LimitRad(yaw_pitch_next.yaw_pitch.x() - yaw_pitch_last.yaw_pitch.x()) /
        (2.0 * PLAN_DEFAULT_DT_S);
    const double pitch_vel =
        (yaw_pitch_next.yaw_pitch.y() - yaw_pitch_last.yaw_pitch.y()) /
        (2.0 * PLAN_DEFAULT_DT_S);
    trajectory.col(index) << LimitRad(yaw_pitch.yaw_pitch.x() - yaw0), yaw_vel,
        yaw_pitch.yaw_pitch.y(), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return true;
}
}  // namespace AimerDetail

/**
 * @brief 初始化 yaw 和 pitch TinyMPC 求解器。
 */
inline void AimerCore::SetupGimbalPlanSolvers()
{
  planner_ready_ = false;
  if (!cfg_.enable_mpc_plan)
  {
    return;
  }

  auto setup_solver = [this](TinySolver** solver, double max_acc, double q_pos,
                             double q_vel, double r_acc) -> bool
  {
    Eigen::MatrixXd a(2, 2);
    a << 1.0, AimerDetail::PLAN_DEFAULT_DT_S, 0.0, 1.0;
    Eigen::MatrixXd b(2, 1);
    b << 0.0, AimerDetail::PLAN_DEFAULT_DT_S;
    Eigen::VectorXd f(2);
    f << 0.0, 0.0;
    Eigen::MatrixXd q(2, 2);
    q << q_pos, 0.0, 0.0, q_vel;
    Eigen::MatrixXd r(1, 1);
    r << r_acc;

    if (tiny_setup(solver, a, b, f, q, r, 1.0, 2, 1,
                   AimerDetail::PLAN_HORIZON, 0) != 0)
    {
      return false;
    }

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(
        2, AimerDetail::PLAN_HORIZON, -1.0e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(
        2, AimerDetail::PLAN_HORIZON, 1.0e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(
        1, AimerDetail::PLAN_HORIZON - 1, -max_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(
        1, AimerDetail::PLAN_HORIZON - 1, max_acc);
    if (tiny_set_bound_constraints(*solver, x_min, x_max, u_min, u_max) != 0)
    {
      return false;
    }

    (*solver)->settings->max_iter = AimerDetail::MPC_MAX_ITER;
    return true;
  };

  const bool yaw_ok =
      setup_solver(&yaw_solver_, cfg_.max_yaw_acc, cfg_.q_yaw_pos, cfg_.q_yaw_vel,
                   cfg_.r_yaw_acc);
  const bool pitch_ok = setup_solver(&pitch_solver_, cfg_.max_pitch_acc,
                                     cfg_.q_pitch_pos, cfg_.q_pitch_vel,
                                     cfg_.r_pitch_acc);
  planner_ready_ = yaw_ok && pitch_ok;
  if (!planner_ready_)
  {
    XR_LOG_WARN("Aimer TinyMPC gimbal_plan setup failed; finite-difference fallback active");
  }
}

/**
 * @brief 在目标状态不连续时清理规划器历史。
 */
inline void AimerCore::ResetGimbalPlanHistory()
{
  last_plan_mpc_ = false;
  ResetFireHold();
}

/**
 * @brief 尝试求解并生成 TinyMPC 云台计划。
 * @param target_msg 当前 tracker 目标。
 * @param bullet_speed 弹速，单位 m/s。
 * @param fire 上游开火门控是否允许开火。
 * @return TinyMPC 产出有限计划时返回 true。
 */
inline bool AimerCore::BuildMpcGimbalPlan(const ArmorTrackerTarget& target_msg,
                                      double delay_time, double bullet_speed,
                                      bool fire)
{
  if (!planner_ready_ || yaw_solver_ == nullptr || pitch_solver_ == nullptr)
  {
    return false;
  }

  AimerDetail::PlanTrajectory reference{};
  double yaw0 = 0.0;
  if (!AimerDetail::BuildReferenceTrajectory(cfg_, target_msg, delay_time,
                                             bullet_speed, lock_id_, reference, yaw0))
  {
    return false;
  }

  Eigen::VectorXd x0(2);
  x0 << reference(0, 0), reference(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = reference.block(0, 0, 2, AimerDetail::PLAN_HORIZON);
  tiny_solve(yaw_solver_);

  x0 << reference(2, 0), reference(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = reference.block(2, 0, 2, AimerDetail::PLAN_HORIZON);
  tiny_solve(pitch_solver_);

  const int output_index = AimerDetail::PLAN_HALF_HORIZON;
  const double target_yaw =
      AimerDetail::LimitRad(reference(0, output_index) + yaw0);
  const double target_pitch = reference(2, output_index);
  const double planned_yaw =
      AimerDetail::LimitRad(yaw_solver_->work->x(0, output_index) + yaw0);
  const double planned_yaw_vel = yaw_solver_->work->x(1, output_index);
  const double planned_yaw_acc = yaw_solver_->work->u(0, output_index);
  const double planned_pitch = pitch_solver_->work->x(0, output_index);
  const double planned_pitch_vel = pitch_solver_->work->x(1, output_index);
  const double planned_pitch_acc = pitch_solver_->work->u(0, output_index);

  if (!std::isfinite(target_yaw) || !std::isfinite(target_pitch) ||
      !std::isfinite(planned_yaw) || !std::isfinite(planned_yaw_vel) ||
      !std::isfinite(planned_yaw_acc) || !std::isfinite(planned_pitch) ||
      !std::isfinite(planned_pitch_vel) || !std::isfinite(planned_pitch_acc))
  {
    return false;
  }

  const int fire_index =
      std::min(AimerDetail::PLAN_HORIZON - 1,
               AimerDetail::PLAN_HALF_HORIZON + AimerDetail::PLAN_SHOOT_OFFSET);
  const double plan_error =
      std::hypot(reference(0, fire_index) - yaw_solver_->work->x(0, fire_index),
                 reference(2, fire_index) - pitch_solver_->work->x(0, fire_index));
  last_fire_plan_error_ = plan_error;
  last_fire_plan_ok_ = plan_error < cfg_.mpc_fire_thresh;

  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  gimbal_plan_msg_.control = true;
  gimbal_plan_msg_.fire = fire && last_fire_plan_ok_;
  gimbal_plan_msg_.target_yaw = static_cast<float>(target_yaw);
  gimbal_plan_msg_.target_pitch = static_cast<float>(target_pitch);
  gimbal_plan_msg_.yaw = static_cast<float>(planned_yaw);
  gimbal_plan_msg_.yaw_vel = static_cast<float>(planned_yaw_vel);
  gimbal_plan_msg_.yaw_acc = static_cast<float>(planned_yaw_acc);
  gimbal_plan_msg_.pitch = static_cast<float>(planned_pitch);
  gimbal_plan_msg_.pitch_vel = static_cast<float>(planned_pitch_vel);
  gimbal_plan_msg_.pitch_acc = static_cast<float>(planned_pitch_acc);
  last_plan_mpc_ = true;
  return true;
}

/**
 * @brief 构建不经过 TinyMPC 平滑的直接 yaw/pitch 云台计划。
 * @param target_msg 当前 tracker 目标。
 * @param control 下级控制器是否应使用该命令。
 * @param fire 是否允许开火。
 * @param yaw 命令 yaw，单位 rad。
 * @param pitch 命令 pitch，单位 rad。
 */
inline void AimerCore::BuildFiniteDifferenceGimbalPlan(
    const ArmorTrackerTarget& target_msg, bool control, bool fire, double yaw,
    double pitch)
{
  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  gimbal_plan_msg_.control = control;
  gimbal_plan_msg_.fire = fire;
  last_plan_mpc_ = false;

  if (!control || !std::isfinite(yaw) || !std::isfinite(pitch))
  {
    ResetGimbalPlanHistory();
    return;
  }

  gimbal_plan_msg_.target_yaw = static_cast<float>(yaw);
  gimbal_plan_msg_.target_pitch = static_cast<float>(pitch);
  gimbal_plan_msg_.yaw = static_cast<float>(yaw);
  gimbal_plan_msg_.pitch = static_cast<float>(pitch);
}

/**
 * @brief 优先选择 TinyMPC 计划，不可用时回退到直接命令输出。
 * @param target_msg 当前 tracker 目标。
 * @param control 下级控制器是否应使用该命令。
 * @param fire 是否允许开火。
 * @param yaw 命令 yaw，单位 rad。
 * @param pitch 命令 pitch，单位 rad。
 * @param bullet_speed 弹速，单位 m/s。
 */
inline void AimerCore::BuildGimbalPlan(const ArmorTrackerTarget& target_msg,
                                   double delay_time, bool control, bool fire,
                                   double yaw, double pitch, double bullet_speed)
{
  last_fire_plan_error_ = std::numeric_limits<double>::quiet_NaN();
  last_fire_plan_ok_ = false;

  if (!control)
  {
    BuildFiniteDifferenceGimbalPlan(target_msg, false, fire, yaw, pitch);
    return;
  }

  if (BuildMpcGimbalPlan(target_msg, delay_time, bullet_speed, fire))
  {
    return;
  }

  BuildFiniteDifferenceGimbalPlan(target_msg, true, fire, yaw, pitch);
}
