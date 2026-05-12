#pragma once

/**
 * @file AimerImpl.hpp
 * @brief Aimer 模块的内联运行时实现。
 */

#include <cmath>
#include <utility>

#include "AimerPlanner.hpp"

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

  auto remember_command = [this, yaw, pitch]()
  {
    last_command_yaw_ = yaw;
    last_command_pitch_ = pitch;
    has_last_command_ = true;
  };

  if (!cfg_.auto_fire || !shootable)
  {
    remember_command();
    return false;
  }

  LibXR::Quaternion<double> gimbal_rotation{};
  bool has_gimbal_rotation = false;
  {
    LibXR::Mutex::LockGuard lock(gimbal_rotation_lock_);
    has_gimbal_rotation = has_gimbal_rotation_;
    gimbal_rotation = gimbal_rotation_;
  }
  if (!has_gimbal_rotation)
  {
    remember_command();
    return false;
  }

  const auto gimbal_euler = gimbal_rotation.ToEulerAngleZYX();
  const double gimbal_yaw = gimbal_euler[2];
  const double gimbal_pitch = gimbal_euler[1];

  if (!has_last_command_)
  {
    remember_command();
    return false;
  }

  const double command_error_yaw =
      std::abs(AimerDetail::LimitRad(last_command_yaw_ - yaw));
  const double command_error_pitch =
      std::abs(AimerDetail::LimitRad(last_command_pitch_ - pitch));
  const double gimbal_error_yaw = std::abs(AimerDetail::LimitRad(gimbal_yaw - yaw));
  const double gimbal_error_pitch =
      std::abs(AimerDetail::LimitRad(gimbal_pitch - pitch));

  const bool command_stable =
      command_error_yaw < yaw_threshold * 2.0 &&
      (!cfg_.enable_pitch_fire_gate || command_error_pitch < pitch_threshold * 2.0);
  const bool gimbal_aligned =
      gimbal_error_yaw < yaw_threshold &&
      (!cfg_.enable_pitch_fire_gate || gimbal_error_pitch < pitch_threshold);

  remember_command();
  return command_stable && gimbal_aligned;
}

/**
 * @brief 处理一帧 tracker 目标并发布全部 Aimer 输出。
 * @param target_msg 当前 tracker 目标消息。
 */
inline void Aimer::TargetCallback(const ArmorTrackerTarget& target_msg)
{
  send_msg_ = {};
  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;

  auto publish_outputs = [&]()
  {
    uint8_t fire_notify = send_msg_.is_fire ? 1U : 0U;
    fire_notify_topic_.Publish(fire_notify);
    gimbal_plan_topic_.Publish(gimbal_plan_msg_);
    send_topic_.Publish(send_msg_);
  };

  if (target_msg.id != last_target_id_)
  {
    lock_id_ = -1;
    has_last_command_ = false;
    last_target_id_ = target_msg.id;
    ResetGimbalPlanHistory();
  }

  double bullet_speed = bullet_speed_.load(std::memory_order_relaxed);
  if (std::isnan(bullet_speed) || bullet_speed < cfg_.min_valid_bullet_speed)
  {
    bullet_speed = cfg_.default_bullet_speed;
  }

  const double delay_time =
      target_msg.tracking ? AimerDetail::FixedPredictDelay(cfg_, target_msg) : 0.0;
  if (!target_msg.tracking)
  {
    has_last_command_ = false;
    ResetGimbalPlanHistory();
    publish_outputs();
    return;
  }

  AimerDetail::PredictedTarget base_target{target_msg};
  base_target.Predict(delay_time);
  AimerDetail::AimPoint aim_point =
      AimerDetail::ChooseAimPoint(base_target, lock_id_);
  if (!aim_point.valid)
  {
    ResetGimbalPlanHistory();
    publish_outputs();
    return;
  }

  const Eigen::Vector3d first_xyz = aim_point.xyza.head<3>();
  const double first_horizontal_distance = AimerDetail::HorizontalDistance(first_xyz);
  const auto first_trajectory = AimerDetail::SolveTrajectoryPitch(
      bullet_speed, first_horizontal_distance, AimerDetail::BallisticHeight(first_xyz));
  if (first_trajectory.unsolvable)
  {
    ResetGimbalPlanHistory();
    publish_outputs();
    return;
  }

  AimerDetail::PredictedTarget hit_target = base_target;
  hit_target.Predict(first_trajectory.fly_time);
  aim_point = AimerDetail::ChooseAimPoint(hit_target, lock_id_);
  if (!aim_point.valid)
  {
    ResetGimbalPlanHistory();
    publish_outputs();
    return;
  }

  const Eigen::Vector3d hit_xyz = aim_point.xyza.head<3>();
  const double hit_horizontal_distance = AimerDetail::HorizontalDistance(hit_xyz);
  const auto trajectory = AimerDetail::SolveTrajectoryPitch(
      bullet_speed, hit_horizontal_distance, AimerDetail::BallisticHeight(hit_xyz));
  if (trajectory.unsolvable)
  {
    ResetGimbalPlanHistory();
    publish_outputs();
    return;
  }

  const Eigen::Vector3d final_xyz = aim_point.xyza.head<3>();
  const double yaw = AimerDetail::LimitRad(
      AimerDetail::BearingYaw(final_xyz) + cfg_.yaw_offset * AimerDetail::DEG2RAD);
  const double pitch = -(trajectory.pitch + cfg_.pitch_offset * AimerDetail::DEG2RAD);

  send_msg_.is_fire =
      ShouldAutoFire(final_xyz, aim_point.view_angle, aim_point.shootable, yaw, pitch);
  send_msg_.position.x() = final_xyz.x();
  send_msg_.position.y() = final_xyz.y();
  send_msg_.position.z() = final_xyz.z();
  send_msg_.v_yaw = target_msg.v_yaw;
  send_msg_.pitch = pitch;
  send_msg_.yaw = yaw;

  BuildGimbalPlan(target_msg, delay_time, true, send_msg_.is_fire, yaw, pitch,
                  bullet_speed);
  publish_outputs();
}
