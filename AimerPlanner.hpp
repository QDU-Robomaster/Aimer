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
/// MPC 开火误差门控使用的采样偏移。
inline constexpr int PLAN_SHOOT_OFFSET = 2;

/// 参考轨迹行含义：yaw 偏移、yaw 速度、roll 轴命令、roll 轴速度。
using PlanTrajectory = Eigen::Matrix<double, 4, PLAN_HORIZON>;

/**
 * @brief 一帧 MPC 计划输出。
 */
struct GimbalPlanSample
{
  double target_yaw{0.0};
  double target_roll{0.0};
  double yaw{0.0};
  double yaw_vel{0.0};
  double yaw_acc{0.0};
  double roll{0.0};
  double roll_vel{0.0};
  double roll_acc{0.0};
};

/**
 * @brief 计算 yaw/roll 两轴计划偏差。
 */
inline double YawRollPlanError(double reference_yaw, double reference_roll,
                                double planned_yaw, double planned_roll)
{
  return std::hypot(LimitRad(reference_yaw - planned_yaw),
                    reference_roll - planned_roll);
}

/**
 * @brief 检查 MPC 输出是否为有限值。
 */
inline bool IsFiniteGimbalPlanSample(const GimbalPlanSample& sample)
{
  return std::isfinite(sample.target_yaw) && std::isfinite(sample.target_roll) &&
         std::isfinite(sample.yaw) && std::isfinite(sample.yaw_vel) &&
         std::isfinite(sample.yaw_acc) && std::isfinite(sample.roll) &&
         std::isfinite(sample.roll_vel) && std::isfinite(sample.roll_acc);
}

/**
 * @brief 计算开火判定采样点。
 */
inline int PlanFireIndex(const AimerConfig& cfg)
{
  int fire_offset =
      static_cast<int>(std::llround(std::max(0.0, cfg.fire_delay_s) /
                                    PLAN_DEFAULT_DT_S));
  if (fire_offset == 0)
  {
    fire_offset = PLAN_SHOOT_OFFSET;
  }
  return std::min(PLAN_HORIZON - 1, PLAN_HALF_HORIZON + fire_offset);
}

/**
 * @brief 构建 TinyMPC 使用的 yaw/roll 参考轨迹。
 * @param cfg Aimer 运行时配置。
 * @param target_msg 当前 tracker 目标。
 * @param delay_time 弹丸飞行前的固定预测延迟，单位 s。
 * @param bullet_speed 弹速，单位 m/s。
 * @param initial_lock_id 参考轨迹的初始锁定面索引。
 * @param trajectory 输出参考轨迹。
 * @param yaw0 输出 yaw 偏置，用于让轨迹数值保持局部。
 * @param fire_target 输出开火采样点对应的预测目标状态。
 * @return 所有参考采样都能计算时返回 true。
 */
inline bool BuildReferenceTrajectory(const AimerConfig& cfg,
                                     const ArmorTrackerTarget& target_msg,
                                     double delay_time, double bullet_speed,
                                     int initial_lock_id,
                                     PlanTrajectory& trajectory, double& yaw0,
                                     PredictedTarget& fire_target)
{
  bool fire_target_ready = false;
  PredictedTarget center_target{target_msg};
  center_target.Predict(delay_time);
  int reference_lock_id = initial_lock_id;

  const auto rough_aim =
      ComputeTrajectoryAimCommand(cfg, center_target, bullet_speed, reference_lock_id);
  if (!rough_aim.valid)
  {
    return false;
  }

  center_target.Predict(rough_aim.fly_time);
  const auto center_aim =
      ComputeTrajectoryAimCommand(cfg, center_target, bullet_speed, reference_lock_id);
  if (!center_aim.valid)
  {
    return false;
  }
  yaw0 = center_aim.yaw_roll.x();

  PredictedTarget moving_target = center_target;
  moving_target.Predict(-PLAN_DEFAULT_DT_S * (PLAN_HALF_HORIZON + 1));
  auto yaw_roll_last =
      ComputeTrajectoryAimCommand(cfg, moving_target, bullet_speed, reference_lock_id);
  if (!yaw_roll_last.valid)
  {
    return false;
  }

  moving_target.Predict(PLAN_DEFAULT_DT_S);
  auto yaw_roll =
      ComputeTrajectoryAimCommand(cfg, moving_target, bullet_speed, reference_lock_id);
  if (!yaw_roll.valid)
  {
    return false;
  }

  const int fire_index = PlanFireIndex(cfg);
  for (int index = 0; index < PLAN_HORIZON; ++index)
  {
    const PredictedTarget sample_target = moving_target;
    moving_target.Predict(PLAN_DEFAULT_DT_S);
    auto yaw_roll_next =
        ComputeTrajectoryAimCommand(cfg, moving_target, bullet_speed, reference_lock_id);
    if (!yaw_roll_next.valid)
    {
      return false;
    }

    const double yaw_vel =
        LimitRad(yaw_roll_next.yaw_roll.x() - yaw_roll_last.yaw_roll.x()) /
        (2.0 * PLAN_DEFAULT_DT_S);
    const double roll_vel =
        (yaw_roll_next.yaw_roll.y() - yaw_roll_last.yaw_roll.y()) /
        (2.0 * PLAN_DEFAULT_DT_S);
    trajectory.col(index) << LimitRad(yaw_roll.yaw_roll.x() - yaw0), yaw_vel,
        yaw_roll.yaw_roll.y(), roll_vel;
    if (index == fire_index)
    {
      fire_target = sample_target;
      fire_target_ready = true;
    }

    yaw_roll_last = yaw_roll;
    yaw_roll = yaw_roll_next;
  }

  return fire_target_ready;
}
}  // namespace AimerDetail

/**
 * @brief 初始化 yaw 和 roll TinyMPC 求解器。
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
  const bool roll_ok = setup_solver(&roll_solver_, cfg_.max_roll_acc,
                                    cfg_.q_roll_pos, cfg_.q_roll_vel,
                                    cfg_.r_roll_acc);
  planner_ready_ = yaw_ok && roll_ok;
  if (!planner_ready_)
  {
    XR_LOG_WARN("Aimer TinyMPC gimbal_plan setup failed; finite-difference fallback active");
  }
}

/**
 * @brief 在目标状态不连续时清理规划器缓存。
 */
inline void AimerCore::ResetGimbalPlanHistory()
{
  last_plan_mpc_ = false;
}

/**
 * @brief 尝试求解并生成 TinyMPC 云台计划。
 * @param target_msg 当前 tracker 目标。
 * @param bullet_speed 弹速，单位 m/s。
 * @return TinyMPC 产出有限计划时返回 true。
 */
inline bool AimerCore::BuildMpcGimbalPlan(const ArmorTrackerTarget& target_msg,
                                          double delay_time, double bullet_speed,
                                          AimerShotCandidate& fire_shot_candidate)
{
  fire_shot_candidate = {};
  if (!planner_ready_ || yaw_solver_ == nullptr || roll_solver_ == nullptr)
  {
    return false;
  }

  AimerDetail::PlanTrajectory reference{};
  double yaw0 = 0.0;
  AimerDetail::PredictedTarget fire_target{};
  if (!AimerDetail::BuildReferenceTrajectory(cfg_, target_msg, delay_time,
                                             bullet_speed, lock_id_, reference, yaw0,
                                             fire_target))
  {
    return false;
  }

  Eigen::VectorXd x0(2);
  x0 << reference(0, 0), reference(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = reference.block(0, 0, 2, AimerDetail::PLAN_HORIZON);
  tiny_solve(yaw_solver_);

  x0 << reference(2, 0), reference(3, 0);
  tiny_set_x0(roll_solver_, x0);
  roll_solver_->work->Xref = reference.block(2, 0, 2, AimerDetail::PLAN_HORIZON);
  tiny_solve(roll_solver_);

  const int output_index = AimerDetail::PLAN_HALF_HORIZON;
  AimerDetail::GimbalPlanSample output{};
  output.target_yaw = AimerDetail::LimitRad(reference(0, output_index) + yaw0);
  output.target_roll = reference(2, output_index);
  output.yaw = AimerDetail::LimitRad(yaw_solver_->work->x(0, output_index) + yaw0);
  output.yaw_vel = yaw_solver_->work->x(1, output_index);
  output.yaw_acc = yaw_solver_->work->u(0, output_index);
  output.roll = roll_solver_->work->x(0, output_index);
  output.roll_vel = roll_solver_->work->x(1, output_index);
  output.roll_acc = roll_solver_->work->u(0, output_index);

  if (!AimerDetail::IsFiniteGimbalPlanSample(output))
  {
    return false;
  }

  const double output_plan_error = AimerDetail::YawRollPlanError(
      output.target_yaw, output.target_roll, output.yaw, output.roll);
  if (output_plan_error > cfg_.mpc_fire_thresh)
  {
    return false;
  }

  const int fire_index = AimerDetail::PlanFireIndex(cfg_);
  const double planned_fire_yaw = AimerDetail::LimitRad(
      yaw_solver_->work->x(0, fire_index) + yaw0);
  const double planned_fire_roll = roll_solver_->work->x(0, fire_index);
  fire_shot_candidate = AimerDetail::ChooseShotCandidateForCommand(
      cfg_, fire_target, bullet_speed, planned_fire_yaw, planned_fire_roll);
  const double shot_plan_error =
      fire_shot_candidate.valid
          ? AimerDetail::YawRollPlanError(
                fire_shot_candidate.yaw, fire_shot_candidate.roll,
                planned_fire_yaw, planned_fire_roll)
          : std::numeric_limits<double>::infinity();

  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  gimbal_plan_msg_.control = true;
  gimbal_plan_msg_.fire = fire_shot_candidate.valid &&
                           fire_shot_candidate.face_shootable_at_hit &&
                           shot_plan_error < cfg_.mpc_fire_thresh;
  gimbal_plan_msg_.target_yaw = static_cast<float>(output.target_yaw);
  gimbal_plan_msg_.target_roll = static_cast<float>(output.target_roll);
  gimbal_plan_msg_.yaw = static_cast<float>(output.yaw);
  gimbal_plan_msg_.yaw_vel = static_cast<float>(output.yaw_vel);
  gimbal_plan_msg_.yaw_acc = static_cast<float>(output.yaw_acc);
  gimbal_plan_msg_.roll = static_cast<float>(output.roll);
  gimbal_plan_msg_.roll_vel = static_cast<float>(output.roll_vel);
  gimbal_plan_msg_.roll_acc = static_cast<float>(output.roll_acc);
  last_plan_mpc_ = true;
  return true;
}

/**
 * @brief 构建不经过 TinyMPC 平滑的直接 yaw/roll 云台计划。
 * @param target_msg 当前 tracker 目标。
 * @param control 下级控制器是否应使用该命令。
 * @param fire_enabled 直接计划是否允许开火。
 * @param yaw 命令 yaw，单位 rad。
 * @param roll 命令 roll 轴，单位 rad。
 */
inline void AimerCore::BuildFiniteDifferenceGimbalPlan(
    const ArmorTrackerTarget& target_msg, bool control, bool fire_enabled,
    double yaw, double roll)
{
  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  gimbal_plan_msg_.control = control;
  gimbal_plan_msg_.fire = fire_enabled;
  last_plan_mpc_ = false;

  if (!control || !std::isfinite(yaw) || !std::isfinite(roll))
  {
    ResetGimbalPlanHistory();
    return;
  }

  gimbal_plan_msg_.target_yaw = static_cast<float>(yaw);
  gimbal_plan_msg_.target_roll = static_cast<float>(roll);
  gimbal_plan_msg_.yaw = static_cast<float>(yaw);
  gimbal_plan_msg_.roll = static_cast<float>(roll);
}

/**
 * @brief 优先选择 TinyMPC 计划，不可用时回退到直接命令输出。
 * @param target_msg 当前 tracker 目标。
 * @param control 下级控制器是否应使用该命令。
 * @param yaw 命令 yaw，单位 rad。
 * @param roll 命令 roll 轴，单位 rad。
 * @param bullet_speed 弹速，单位 m/s。
 */
inline void AimerCore::BuildGimbalPlan(const ArmorTrackerTarget& target_msg,
                                       double delay_time, bool control,
                                       double yaw, double roll, double bullet_speed,
                                       const AimerShotCandidate& direct_shot_candidate,
                                       AimerShotCandidate& fire_shot_candidate)
{
  fire_shot_candidate = direct_shot_candidate;
  if (!control)
  {
    BuildFiniteDifferenceGimbalPlan(target_msg, false, false, yaw, roll);
    return;
  }

  if (BuildMpcGimbalPlan(target_msg, delay_time, bullet_speed,
                         fire_shot_candidate))
  {
    return;
  }

  fire_shot_candidate = direct_shot_candidate;
  BuildFiniteDifferenceGimbalPlan(
      target_msg, true,
      direct_shot_candidate.valid && direct_shot_candidate.face_shootable_at_hit,
      yaw, roll);
}
