#pragma once

/**
 * @file AimerImpl.hpp
 * @brief Aimer 模块的内联运行时实现。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

#include "AimerAudit.hpp"
#include "AimerPlanner.hpp"
#include "logger.hpp"

/**
 * @brief 构造 Aimer 并注册 tracker、referee、gimbal 输入 topic 回调。
 */
inline Aimer::Aimer(LibXR::HardwareContainer&, LibXR::ApplicationManager& app,
                    Config cfg)
    : cfg_(std::move(cfg)), bullet_speed_(cfg_.default_bullet_speed)
{
  SetupGimbalPlanSolvers();

  LibXR::Topic::Domain tracker_domain("tracker");
  LibXR::Topic target_topic =
      LibXR::Topic::FindOrCreate<ArmorTrackerTarget>("target", &tracker_domain);
  auto target_callback = LibXR::Topic::Callback::Create(
      [](bool, Aimer* self, LibXR::RawData& data)
      {
        auto* target_msg = reinterpret_cast<ArmorTrackerTarget*>(data.addr_);
        self->TargetCallback(*target_msg);
      },
      this);
  target_topic.RegisterCallback(target_callback);

  LibXR::Topic::Domain referee_domain("referee");
  LibXR::Topic bullet_speed_topic =
      LibXR::Topic::FindOrCreate<float>("bullet_speed", &referee_domain);
  auto bullet_speed_callback = LibXR::Topic::Callback::Create(
      [](bool, Aimer* self, LibXR::RawData& data)
      {
        auto* bullet_speed_msg = reinterpret_cast<float*>(data.addr_);
        self->BulletSpeedCallback(*bullet_speed_msg);
      },
      this);
  bullet_speed_topic.RegisterCallback(bullet_speed_callback);

  LibXR::Topic::Domain gimbal_domain("gimbal");
  LibXR::Topic gimbal_rotation_topic =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation", &gimbal_domain);
  auto gimbal_rotation_callback = LibXR::Topic::Callback::Create(
      [](bool, Aimer* self, LibXR::RawData& data)
      {
        auto* gimbal_rotation_msg =
            reinterpret_cast<LibXR::Quaternion<float>*>(data.addr_);
        self->GimbalRotationCallback(*gimbal_rotation_msg);
      },
      this);
  gimbal_rotation_topic.RegisterCallback(gimbal_rotation_callback);

  if (const char* env = std::getenv("XR_AIMER_DECISION_TSV"))
  {
    if (env[0] != '\0')
    {
      decision_audit_.path = env;
    }
  }
  if (const char* env = std::getenv("XR_AIMER_SHOT_TSV"))
  {
    if (env[0] != '\0')
    {
      shot_audit_.path = env;
    }
  }

  app.Register(*this);
}

/**
 * @brief 根据裁判系统弹速消息更新内部弹速缓存。
 * @param bullet_speed_msg 最新弹速，单位 m/s。
 */
inline void Aimer::BulletSpeedCallback(float bullet_speed_msg)
{
  if (!std::isnan(bullet_speed_msg))
  {
    bullet_speed_.store(bullet_speed_msg, std::memory_order_relaxed);
  }
}

/**
 * @brief 根据云台姿态消息更新内部姿态缓存。
 * @param gimbal_rotation_msg 云台姿态四元数。
 */
inline void Aimer::GimbalRotationCallback(LibXR::Quaternion<float> gimbal_rotation_msg)
{
  LibXR::Mutex::LockGuard lock(gimbal_rotation_lock_);
  gimbal_rotation_ =
      LibXR::Quaternion<double>(gimbal_rotation_msg.w(), gimbal_rotation_msg.x(),
                                gimbal_rotation_msg.y(), gimbal_rotation_msg.z());
  has_gimbal_rotation_ = true;
}

/**
 * @brief 判断当前命令是否满足自动开火条件。
 * @param target_xyz tracker 相机坐标系下的目标瞄点。
 * @param selected_view_angle 选中装甲板相对整车中心方位的视角。
 * @param shootable 当前策略是否允许打该装甲板。
 * @param yaw 命令 yaw，单位 rad。
 * @param pitch 命令 pitch，单位 rad。
 * @return 所有开火门控通过时返回 true。
 */
inline bool Aimer::ShouldAutoFire(const Eigen::Vector3d& target_xyz,
                                  double selected_view_angle, bool shootable,
                                  double yaw, double pitch)
{
  const auto [yaw_threshold, pitch_threshold] =
      AimerDetail::DynamicFireThreshold(cfg_, target_xyz, selected_view_angle);
  last_fire_tolerance_rad_ = yaw_threshold;
  metrics_msg_.fire_thres_yaw = yaw_threshold;
  metrics_msg_.fire_thres_pitch = pitch_threshold;
  decision_msg_.fire_thres_yaw = yaw_threshold;
  decision_msg_.fire_thres_pitch = pitch_threshold;

  if (!cfg_.auto_fire)
  {
    last_fire_command_error_rad_ = 0.0;
    last_fire_command_pitch_error_rad_ = 0.0;
    last_fire_gimbal_error_rad_ = 0.0;
    last_fire_gimbal_pitch_error_rad_ = 0.0;
    last_fire_gimbal_yaw_rad_ = 0.0;
    last_fire_gimbal_pitch_rad_ = 0.0;
    metrics_msg_.fire_reason = FireReason::DISABLED;
    last_command_yaw_ = yaw;
    last_command_pitch_ = pitch;
    has_last_command_ = true;
    return false;
  }

  if (!shootable)
  {
    metrics_msg_.fire_reason = FireReason::NOT_SHOOTABLE;
    last_command_yaw_ = yaw;
    last_command_pitch_ = pitch;
    has_last_command_ = true;
    return false;
  }
  LibXR::Quaternion<double> gimbal_rotation{};
  bool has_gimbal_rotation = false;
  double gimbal_yaw = 0.0;
  {
    LibXR::Mutex::LockGuard lock(gimbal_rotation_lock_);
    has_gimbal_rotation = has_gimbal_rotation_;
    gimbal_rotation = gimbal_rotation_;
  }
  if (!has_gimbal_rotation)
  {
    last_fire_command_error_rad_ = 0.0;
    last_fire_command_pitch_error_rad_ = 0.0;
    last_fire_gimbal_error_rad_ = 0.0;
    last_fire_gimbal_pitch_error_rad_ = 0.0;
    last_fire_gimbal_yaw_rad_ = 0.0;
    last_fire_gimbal_pitch_rad_ = 0.0;
    metrics_msg_.fire_reason = FireReason::NO_GIMBAL;
    last_command_yaw_ = yaw;
    last_command_pitch_ = pitch;
    has_last_command_ = true;
    return false;
  }

  const auto gimbal_euler = gimbal_rotation.ToEulerAngleZYX();
  gimbal_yaw = gimbal_euler[2];
  const double gimbal_pitch = gimbal_euler[1];
  last_fire_gimbal_yaw_rad_ = gimbal_yaw;
  last_fire_gimbal_pitch_rad_ = gimbal_pitch;

  if (!has_last_command_)
  {
    last_fire_command_error_rad_ = std::numeric_limits<double>::infinity();
    last_fire_command_pitch_error_rad_ = std::numeric_limits<double>::infinity();
    last_fire_gimbal_error_rad_ = std::abs(AimerDetail::LimitRad(gimbal_yaw - yaw));
    last_fire_gimbal_pitch_error_rad_ =
        std::abs(AimerDetail::LimitRad(gimbal_pitch - pitch));
    metrics_msg_.fire_reason = FireReason::COMMAND_UNSTABLE;
    last_command_yaw_ = yaw;
    last_command_pitch_ = pitch;
    has_last_command_ = true;
    return false;
  }

  last_fire_command_error_rad_ =
      std::abs(AimerDetail::LimitRad(last_command_yaw_ - yaw));
  last_fire_command_pitch_error_rad_ =
      std::abs(AimerDetail::LimitRad(last_command_pitch_ - pitch));
  last_fire_gimbal_error_rad_ = std::abs(AimerDetail::LimitRad(gimbal_yaw - yaw));
  last_fire_gimbal_pitch_error_rad_ =
      std::abs(AimerDetail::LimitRad(gimbal_pitch - pitch));
  metrics_msg_.command_error_yaw = last_fire_command_error_rad_;
  metrics_msg_.command_error_pitch = last_fire_command_pitch_error_rad_;
  metrics_msg_.gimbal_error_yaw = last_fire_gimbal_error_rad_;
  metrics_msg_.gimbal_error_pitch = last_fire_gimbal_pitch_error_rad_;
  decision_msg_.command_error_yaw = last_fire_command_error_rad_;
  decision_msg_.command_error_pitch = last_fire_command_pitch_error_rad_;
  decision_msg_.actual_gimbal_error_yaw = last_fire_gimbal_error_rad_;
  decision_msg_.actual_gimbal_error_pitch = last_fire_gimbal_pitch_error_rad_;

  const bool command_stable =
      last_fire_command_error_rad_ < yaw_threshold * 2.0 &&
      (!cfg_.enable_pitch_fire_gate ||
       last_fire_command_pitch_error_rad_ < pitch_threshold * 2.0);
  const bool gimbal_aligned =
      last_fire_gimbal_error_rad_ < yaw_threshold &&
      (!cfg_.enable_pitch_fire_gate ||
       last_fire_gimbal_pitch_error_rad_ < pitch_threshold);

  last_command_yaw_ = yaw;
  last_command_pitch_ = pitch;
  has_last_command_ = true;
  if (!command_stable)
  {
    metrics_msg_.fire_reason = FireReason::COMMAND_UNSTABLE;
    return false;
  }
  if (!gimbal_aligned)
  {
    metrics_msg_.fire_reason = FireReason::GIMBAL_NOT_ALIGNED;
    return false;
  }
  metrics_msg_.fire_reason = FireReason::OK;
  return true;
}

/**
 * @brief 构建当前帧的调试弹道消息。
 * @param target_msg 当前 tracker 目标。
 * @param aim_point tracker 相机坐标系下的最终瞄点。
 * @param fly_time 估计飞行时间，单位 s。
 * @param launch_pitch 加配置 pitch 偏置前的发射 pitch，单位 rad。
 * @param bullet_speed 弹速，单位 m/s。
 * @param yaw 命令 yaw，单位 rad。
 * @param pitch 命令 pitch，单位 rad。
 */
inline void Aimer::BuildTrajectoryMessage(const ArmorTrackerTarget& target_msg,
                                          const Eigen::Vector3d& aim_point,
                                          double fly_time, double launch_pitch,
                                          double bullet_speed, double yaw,
                                          double pitch)
{
  trajectory_msg_ = {};
  trajectory_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  trajectory_msg_.valid = true;
  trajectory_msg_.converged = metrics_msg_.converged;
  trajectory_msg_.selected_armor_index =
      static_cast<uint8_t>(std::min<uint32_t>(metrics_msg_.selected_armor_index, 255U));
  trajectory_msg_.target_id = target_msg.id;
  trajectory_msg_.bullet_speed = bullet_speed;
  trajectory_msg_.delay_time_s = metrics_msg_.delay_time_s;
  trajectory_msg_.fly_time_s = fly_time;
  trajectory_msg_.yaw = yaw;
  trajectory_msg_.pitch = pitch;
  trajectory_msg_.aim_point =
      LibXR::Position<double>(aim_point.x(), aim_point.y(), aim_point.z());

  const double horizontal_distance = AimerDetail::HorizontalDistance(aim_point);
  if (fly_time <= 0.0 || bullet_speed <= 0.0 ||
      horizontal_distance <= AimerDetail::MIN_HORIZONTAL_DISTANCE_M)
  {
    trajectory_msg_.valid = false;
    return;
  }

  const double dir_x = aim_point.x() / horizontal_distance;
  const double dir_z = aim_point.z() / horizontal_distance;
  const double v_horizontal = bullet_speed * std::cos(launch_pitch);
  const double v_vertical = bullet_speed * std::sin(launch_pitch);
  trajectory_msg_.point_count = AimerTrajectory::MAX_POINTS;
  for (uint8_t index = 0; index < AimerTrajectory::MAX_POINTS; ++index)
  {
    const double ratio = static_cast<double>(index) /
                         static_cast<double>(AimerTrajectory::MAX_POINTS - 1U);
    const double t = fly_time * ratio;
    const double s = v_horizontal * t;
    const double y = v_vertical * t - 0.5 * AimerDetail::GRAVITY * t * t;
    trajectory_msg_.points[index] =
        LibXR::Position<double>(dir_x * s, y, dir_z * s);
  }
}

/**
 * @brief 发布 decision，并在本帧允许开火时发布 shot_event。
 */
inline void Aimer::PublishDecisionAndMaybeShot()
{
  decision_msg_.target_yaw = gimbal_plan_msg_.target_yaw;
  decision_msg_.target_pitch = gimbal_plan_msg_.target_pitch;
  decision_msg_.planned_yaw = gimbal_plan_msg_.yaw;
  decision_msg_.planned_pitch = gimbal_plan_msg_.pitch;
  decision_msg_.planned_yaw_vel = gimbal_plan_msg_.yaw_vel;
  decision_msg_.planned_pitch_vel = gimbal_plan_msg_.pitch_vel;
  decision_msg_.planned_yaw_acc = gimbal_plan_msg_.yaw_acc;
  decision_msg_.planned_pitch_acc = gimbal_plan_msg_.pitch_acc;
  decision_msg_.mpc_used = last_plan_mpc_;
  decision_msg_.fire_allowed = send_msg_.is_fire;
  decision_msg_.fire_reason = metrics_msg_.fire_reason;
  decision_topic_.Publish(decision_msg_);
  WriteDecisionAudit();

  if (!send_msg_.is_fire)
  {
    return;
  }

  AimerShotEvent shot{};
  shot.shot_id = ++shot_index_;
  shot.frame_id = decision_msg_.frame_id;
  shot.image_timestamp_us = decision_msg_.image_timestamp_us;
  shot.command_time_us = decision_msg_.aimer_receive_time_us;
  shot.expected_hit_time_us = decision_msg_.expected_hit_time_us;
  shot.selected_armor_index = decision_msg_.selected_armor_index;
  shot.target_id = decision_msg_.target_id;
  shot.command_yaw = decision_msg_.command_yaw;
  shot.command_pitch = decision_msg_.command_pitch;
  shot.actual_gimbal_yaw = last_fire_gimbal_yaw_rad_;
  shot.actual_gimbal_pitch = last_fire_gimbal_pitch_rad_;
  shot.bullet_speed = metrics_msg_.bullet_speed;
  shot.fire_delay_s = cfg_.fire_delay_s;
  shot.fly_time_est_s = metrics_msg_.fly_time_s;
  shot.fire_reason = metrics_msg_.fire_reason;
  shot_event_topic_.Publish(shot);
  WriteShotAudit(shot);
}

/**
 * @brief 处理一帧 tracker 目标并发布全部 Aimer 输出。
 * @param target_msg 当前 tracker 目标消息。
 */
inline void Aimer::TargetCallback(const ArmorTrackerTarget& target_msg)
{
  const auto start_time = std::chrono::steady_clock::now();
  const uint64_t receive_time_us = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          start_time.time_since_epoch())
          .count());
  auto publish_outputs = [&](bool publish_target_euler)
  {
    uint8_t fire_notify = send_msg_.is_fire ? 1U : 0U;
    metrics_msg_.latency_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - start_time)
                                  .count();
    metrics_topic_.Publish(metrics_msg_);
    trajectory_topic_.Publish(trajectory_msg_);
    if (publish_target_euler)
    {
      target_euler_topic_.Publish(target_euler_msg_);
    }
    fire_notify_topic_.Publish(fire_notify);
    gimbal_plan_topic_.Publish(gimbal_plan_msg_);
    send_topic_.Publish(send_msg_);
    PublishDecisionAndMaybeShot();
  };

  ++frame_index_;
  metrics_msg_ = {};
  metrics_msg_.frame_index = frame_index_;
  metrics_msg_.target_tracking = target_msg.tracking;
  metrics_msg_.target_id = target_msg.id;
  metrics_msg_.strategy = AimerDetail::SelectStrategy(cfg_, target_msg);
  metrics_msg_.fire_reason = FireReason::NO_TRACK;
  trajectory_msg_ = {};
  trajectory_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  trajectory_msg_.target_id = target_msg.id;
  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  decision_msg_ = {};
  decision_msg_.frame_id = frame_index_;
  decision_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  decision_msg_.aimer_receive_time_us = receive_time_us;
  decision_msg_.target_tracking = target_msg.tracking;
  decision_msg_.target_id = target_msg.id;
  decision_msg_.strategy = metrics_msg_.strategy;
  decision_msg_.fire_reason = metrics_msg_.fire_reason;

  if (target_msg.id != last_target_id_)
  {
    lock_id_ = -1;
    has_last_command_ = false;
    last_target_id_ = target_msg.id;
    ResetGimbalPlanHistory();
    metrics_msg_.switch_reason = SwitchReason::NEW_TARGET;
    decision_msg_.switch_reason = SwitchReason::NEW_TARGET;
  }

  send_msg_ = {};
  target_euler_msg_ = LibXR::EulerAngle<float>();

  double bullet_speed = bullet_speed_.load(std::memory_order_relaxed);
  if (std::isnan(bullet_speed) || bullet_speed < cfg_.min_valid_bullet_speed)
  {
    bullet_speed = cfg_.default_bullet_speed;
  }
  metrics_msg_.bullet_speed = bullet_speed;

  const double delay_time =
      target_msg.tracking ? AimerDetail::FixedPredictDelay(cfg_, target_msg) : 0.0;
  metrics_msg_.delay_time_s = delay_time;
  decision_msg_.fixed_delay_s = delay_time;
  decision_msg_.fire_delay_s = cfg_.fire_delay_s;
  decision_msg_.predict_time_us =
      target_msg.image_timestamp_us + AimerDetail::SecondsToMicros(delay_time);

  if (!target_msg.tracking)
  {
    has_last_command_ = false;
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  AimerDetail::PredictedTarget base_target{target_msg};
  base_target.Predict(delay_time);

  auto record_aim_point = [this](const AimerDetail::AimPoint& aim_point)
  {
    metrics_msg_.strategy = aim_point.strategy;
    metrics_msg_.selected_reason = aim_point.selected_reason;
    if (metrics_msg_.switch_reason == SwitchReason::NONE)
    {
      metrics_msg_.switch_reason = aim_point.switch_reason;
    }
    metrics_msg_.selected_armor_index =
        static_cast<uint32_t>(std::max(aim_point.armor_index, 0));

    decision_msg_.strategy = aim_point.strategy;
    decision_msg_.selected_reason = aim_point.selected_reason;
    if (decision_msg_.switch_reason == SwitchReason::NONE)
    {
      decision_msg_.switch_reason = aim_point.switch_reason;
    }
    decision_msg_.candidate_count = aim_point.candidate_count;
    decision_msg_.selected_armor_index =
        static_cast<uint8_t>(std::clamp(aim_point.armor_index, 0, 255));
    decision_msg_.selected_x = aim_point.xyza.x();
    decision_msg_.selected_y = aim_point.xyza.y();
    decision_msg_.selected_z = aim_point.xyza.z();
    decision_msg_.selected_yaw = aim_point.xyza.w();
    decision_msg_.selected_view_angle = aim_point.view_angle;
    decision_msg_.selected_front_facing = aim_point.front_facing;
    decision_msg_.shootable = aim_point.shootable;
  };

  AimerDetail::AimPoint debug_aim_point =
      AimerDetail::ChooseAimPoint(cfg_, base_target, lock_id_);

  if (!debug_aim_point.valid)
  {
    metrics_msg_.fire_reason = FireReason::NOT_SHOOTABLE;
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }
  record_aim_point(debug_aim_point);

  const Eigen::Vector3d first_xyz = debug_aim_point.xyza.head<3>();
  const double first_horizontal_distance = AimerDetail::HorizontalDistance(first_xyz);
  const auto first_trajectory = AimerDetail::SolveTrajectoryPitch(
      bullet_speed, first_horizontal_distance, AimerDetail::BallisticHeight(first_xyz));

  if (first_trajectory.unsolvable)
  {
    metrics_msg_.fire_reason = FireReason::BALLISTIC_UNSOLVABLE;
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  AimerDetail::PredictedTarget hit_target = base_target;
  hit_target.Predict(first_trajectory.fly_time);
  debug_aim_point = AimerDetail::ChooseAimPoint(cfg_, hit_target, lock_id_);
  if (!debug_aim_point.valid)
  {
    metrics_msg_.fire_reason = FireReason::NOT_SHOOTABLE;
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  record_aim_point(debug_aim_point);

  const Eigen::Vector3d hit_xyz = debug_aim_point.xyza.head<3>();
  const double hit_horizontal_distance = AimerDetail::HorizontalDistance(hit_xyz);
  const auto trajectory = AimerDetail::SolveTrajectoryPitch(
      bullet_speed, hit_horizontal_distance, AimerDetail::BallisticHeight(hit_xyz));
  if (trajectory.unsolvable)
  {
    metrics_msg_.fire_reason = FireReason::BALLISTIC_UNSOLVABLE;
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  constexpr bool converged = true;
  metrics_msg_.iteration_count = 1;

  metrics_msg_.valid = true;
  metrics_msg_.converged = converged;
  metrics_msg_.fly_time_s = first_trajectory.fly_time;
  metrics_msg_.total_hit_delay_s =
      delay_time + cfg_.fire_delay_s + first_trajectory.fly_time;
  decision_msg_.valid = true;
  decision_msg_.converged = converged;
  decision_msg_.fly_time_s = first_trajectory.fly_time;
  decision_msg_.total_hit_delay_s = metrics_msg_.total_hit_delay_s;
  decision_msg_.expected_hit_time_us =
      target_msg.image_timestamp_us +
      AimerDetail::SecondsToMicros(metrics_msg_.total_hit_delay_s);

  const Eigen::Vector3d final_xyz = debug_aim_point.xyza.head<3>();
  const double yaw = AimerDetail::LimitRad(
      AimerDetail::BearingYaw(final_xyz) + cfg_.yaw_offset * AimerDetail::DEG2RAD);
  const double pitch = -(trajectory.pitch + cfg_.pitch_offset * AimerDetail::DEG2RAD);

  metrics_msg_.yaw = yaw;
  metrics_msg_.pitch = pitch;
  decision_msg_.command_yaw = yaw;
  decision_msg_.command_pitch = pitch;

  target_euler_msg_.Pitch() = static_cast<float>(pitch);
  target_euler_msg_.Yaw() = static_cast<float>(yaw);

  send_msg_.is_fire =
      ShouldAutoFire(final_xyz, debug_aim_point.view_angle, debug_aim_point.shootable,
                     yaw, pitch);
  BuildTrajectoryMessage(target_msg, final_xyz, trajectory.fly_time, trajectory.pitch,
                         bullet_speed, yaw, pitch);
  trajectory_msg_.fire = send_msg_.is_fire;
  send_msg_.position.x() = final_xyz.x();
  send_msg_.position.y() = final_xyz.y();
  send_msg_.position.z() = final_xyz.z();
  send_msg_.v_yaw = target_msg.v_yaw;
  send_msg_.pitch = pitch;
  send_msg_.yaw = yaw;
  metrics_msg_.is_fire = send_msg_.is_fire;
  metrics_msg_.planner_mpc = false;
  BuildGimbalPlan(target_msg, true, send_msg_.is_fire, yaw, pitch, bullet_speed);
  metrics_msg_.planner_mpc = last_plan_mpc_;

  publish_outputs(true);

  if ((frame_index_ % 30U) == 0U)
  {
    XR_LOG_INFO(
        "Aimer frame=%llu target=%s valid=%d converged=%d fire=%d iter=%u strategy=%s "
        "select=%s switch=%s fire_reason=%s delay_s=%.3f fly_s=%.3f yaw=%.3f "
        "gimbal_yaw=%.3f cmd_err_deg=%.2f gimbal_err_deg=%.2f tol_deg=%.2f "
        "plan_mpc=%d plan_yaw=%.3f plan_pitch=%.3f "
        "plan_yaw_vel=%.3f plan_pitch_vel=%.3f plan_yaw_acc=%.3f "
        "plan_pitch_acc=%.3f latency_ms=%.2f",
        static_cast<unsigned long long>(frame_index_),
        AimerDetail::ArmorNumberToString(target_msg.id).data(),
        metrics_msg_.valid ? 1 : 0, metrics_msg_.converged ? 1 : 0,
        metrics_msg_.is_fire ? 1 : 0, metrics_msg_.iteration_count,
        AimerDetail::ToString(metrics_msg_.strategy),
        AimerDetail::ToString(metrics_msg_.selected_reason),
        AimerDetail::ToString(metrics_msg_.switch_reason),
        AimerDetail::ToString(metrics_msg_.fire_reason), metrics_msg_.delay_time_s,
        metrics_msg_.fly_time_s, metrics_msg_.yaw, last_fire_gimbal_yaw_rad_,
        last_fire_command_error_rad_ / AimerDetail::DEG2RAD,
        last_fire_gimbal_error_rad_ / AimerDetail::DEG2RAD,
        last_fire_tolerance_rad_ / AimerDetail::DEG2RAD, last_plan_mpc_ ? 1 : 0,
        static_cast<double>(gimbal_plan_msg_.yaw),
        static_cast<double>(gimbal_plan_msg_.pitch),
        static_cast<double>(gimbal_plan_msg_.yaw_vel),
        static_cast<double>(gimbal_plan_msg_.pitch_vel),
        static_cast<double>(gimbal_plan_msg_.yaw_acc),
        static_cast<double>(gimbal_plan_msg_.pitch_acc), metrics_msg_.latency_ms);
  }
}
