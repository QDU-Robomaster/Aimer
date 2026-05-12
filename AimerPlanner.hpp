#pragma once

/**
 * @file AimerPlanner.hpp
 * @brief TinyMPC and finite-difference gimbal plan generation.
 */

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "AimerTargetModel.hpp"
#include "logger.hpp"
#include "tinympc/tiny_api.hpp"

namespace AimerDetail
{
inline constexpr double PLAN_DEFAULT_DT_S = 0.01;
inline constexpr int PLAN_HALF_HORIZON = 50;
inline constexpr int PLAN_HORIZON = PLAN_HALF_HORIZON * 2;
inline constexpr int PLAN_SHOOT_OFFSET = 2;

using PlanTrajectory = Eigen::Matrix<double, 4, PLAN_HORIZON>;

inline bool BuildReferenceTrajectory(const Aimer::Config& cfg,
                                     const ArmorTrackerTarget& target_msg,
                                     double delay_time, double bullet_speed,
                                     PlanTrajectory& trajectory, double& yaw0)
{
  PredictedTarget center_target{target_msg};
  center_target.Predict(delay_time);

  const auto rough_aim = ComputeNearestAimCommand(cfg, center_target, bullet_speed);
  if (!rough_aim.valid)
  {
    return false;
  }

  center_target.Predict(rough_aim.fly_time);
  const auto center_aim = ComputeNearestAimCommand(cfg, center_target, bullet_speed);
  if (!center_aim.valid)
  {
    return false;
  }
  yaw0 = center_aim.yaw_pitch.x();

  PredictedTarget moving_target = center_target;
  moving_target.Predict(-PLAN_DEFAULT_DT_S * (PLAN_HALF_HORIZON + 1));
  auto yaw_pitch_last = ComputeNearestAimCommand(cfg, moving_target, bullet_speed);
  if (!yaw_pitch_last.valid)
  {
    return false;
  }

  moving_target.Predict(PLAN_DEFAULT_DT_S);
  auto yaw_pitch = ComputeNearestAimCommand(cfg, moving_target, bullet_speed);
  if (!yaw_pitch.valid)
  {
    return false;
  }

  for (int index = 0; index < PLAN_HORIZON; ++index)
  {
    moving_target.Predict(PLAN_DEFAULT_DT_S);
    auto yaw_pitch_next = ComputeNearestAimCommand(cfg, moving_target, bullet_speed);
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

inline void Aimer::SetupGimbalPlanSolvers()
{
  planner_ready_ = false;
  if (!cfg_.enable_mpc_plan)
  {
    XR_LOG_INFO("Aimer TinyMPC gimbal_plan disabled by config");
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

    (*solver)->settings->max_iter = cfg_.mpc_max_iter;
    return true;
  };

  const bool yaw_ok =
      setup_solver(&yaw_solver_, cfg_.max_yaw_acc, cfg_.q_yaw_pos, cfg_.q_yaw_vel,
                   cfg_.r_yaw_acc);
  const bool pitch_ok = setup_solver(&pitch_solver_, cfg_.max_pitch_acc,
                                     cfg_.q_pitch_pos, cfg_.q_pitch_vel,
                                     cfg_.r_pitch_acc);
  planner_ready_ = yaw_ok && pitch_ok;
  if (planner_ready_)
  {
    XR_LOG_INFO(
        "Aimer TinyMPC gimbal_plan enabled horizon=%d dt=%.3f max_yaw_acc=%.1f max_pitch_acc=%.1f iter=%d",
        AimerDetail::PLAN_HORIZON, AimerDetail::PLAN_DEFAULT_DT_S,
        cfg_.max_yaw_acc, cfg_.max_pitch_acc, cfg_.mpc_max_iter);
  }
  else
  {
    XR_LOG_WARN("Aimer TinyMPC gimbal_plan setup failed; finite-difference fallback active");
  }
}

inline void Aimer::ResetGimbalPlanHistory()
{
  last_plan_mpc_ = false;
}

inline bool Aimer::BuildMpcGimbalPlan(const ArmorTrackerTarget& target_msg,
                                      double bullet_speed, bool fire)
{
  if (!planner_ready_ || yaw_solver_ == nullptr || pitch_solver_ == nullptr)
  {
    return false;
  }

  AimerDetail::PlanTrajectory reference{};
  double yaw0 = 0.0;
  if (!AimerDetail::BuildReferenceTrajectory(cfg_, target_msg,
                                             metrics_msg_.delay_time_s,
                                             bullet_speed, reference, yaw0))
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

  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  gimbal_plan_msg_.control = true;
  gimbal_plan_msg_.fire = fire && plan_error < cfg_.mpc_fire_thresh;
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

inline void Aimer::BuildFiniteDifferenceGimbalPlan(
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

inline void Aimer::BuildGimbalPlan(const ArmorTrackerTarget& target_msg,
                                   bool control, bool fire, double yaw,
                                   double pitch, double bullet_speed)
{
  if (!control)
  {
    BuildFiniteDifferenceGimbalPlan(target_msg, false, fire, yaw, pitch);
    return;
  }

  if (BuildMpcGimbalPlan(target_msg, bullet_speed, fire))
  {
    return;
  }

  BuildFiniteDifferenceGimbalPlan(target_msg, true, fire, yaw, pitch);
}
