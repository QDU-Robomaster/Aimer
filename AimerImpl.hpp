#pragma once

/**
 * @file AimerImpl.hpp
 * @brief Aimer 模块的内联运行时实现。
 */

#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>

#include "AimerPlanner.hpp"

/**
 * @brief 构造 Aimer 并注册 tracker、referee、gimbal 输入 topic 回调。
 */
inline AimerCore::AimerCore(LibXR::HardwareContainer&, LibXR::ApplicationManager& app,
                    Config cfg)
    : cfg_(std::move(cfg)), bullet_speed_(cfg_.default_bullet_speed)
{
  SetupGimbalPlanSolvers();
  RegisterRuntimeLogCallbacks();

  LibXR::Topic::Domain tracker_domain("tracker");
  LibXR::Topic target_topic =
      LibXR::Topic::FindOrCreate<ArmorTrackerTarget>("target", &tracker_domain);
  auto target_callback = LibXR::Topic::Callback::Create(
      [](bool, AimerCore* self, LibXR::RawData& data)
      {
        auto* target_msg = reinterpret_cast<ArmorTrackerTarget*>(data.addr_);
        if (target_msg != nullptr && data.size_ == sizeof(ArmorTrackerTarget))
        {
          self->TargetCallback(*target_msg);
        }
      },
      this);
  target_topic.RegisterCallback(target_callback);

  LibXR::Topic::Domain gimbal_domain("gimbal");
  LibXR::Topic gimbal_rotation_topic =
      LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("rotation", &gimbal_domain);
  auto gimbal_rotation_callback = LibXR::Topic::Callback::Create(
      [](bool, AimerCore* self, LibXR::RawData& data)
      {
        auto* gimbal_rotation_msg =
            reinterpret_cast<LibXR::Quaternion<float>*>(data.addr_);
        if (gimbal_rotation_msg != nullptr &&
            data.size_ == sizeof(LibXR::Quaternion<float>))
        {
          self->GimbalRotationCallback(*gimbal_rotation_msg);
        }
      },
      this);
  gimbal_rotation_topic.RegisterCallback(gimbal_rotation_callback);

  app.Register(*this);
}

/**
 * @brief 注册裁判系统运行期回调。
 */
inline void AimerCore::RegisterRuntimeLogCallbacks()
{
  LibXR::Topic::Domain referee_domain("host");
  LibXR::Topic referee_topic =
      LibXR::Topic::FindOrCreate<AimerRefereeSummary>("robot_game_ref",
                                                      &referee_domain);
  auto referee_callback = LibXR::Topic::Callback::Create(
      [](bool, AimerCore* self, LibXR::RawData& data)
      {
        auto* summary = reinterpret_cast<AimerRefereeSummary*>(data.addr_);
        if (summary != nullptr && data.size_ == sizeof(AimerRefereeSummary))
        {
          self->RefereeSummaryCallback(*summary);
        }
      },
      this);
  referee_topic.RegisterCallback(referee_callback);
}

/**
 * @brief 统一更新弹速缓存并按变化量输出日志。
 */
inline void AimerCore::UpdateBulletSpeed(float bullet_speed_msg, const char* source)
{
  if (!std::isfinite(bullet_speed_msg))
  {
    return;
  }

  const double new_bullet_speed = static_cast<double>(bullet_speed_msg);
  const double old_bullet_speed =
      bullet_speed_.exchange(new_bullet_speed, std::memory_order_relaxed);

  if (!cfg_.enable_runtime_log)
  {
    return;
  }

  LibXR::Mutex::LockGuard lock(runtime_log_lock_);
  const bool should_log =
      !have_logged_bullet_speed_ ||
      std::abs(new_bullet_speed - last_logged_bullet_speed_) >=
          std::max(0.0, cfg_.bullet_speed_log_delta);
  if (!should_log)
  {
    return;
  }

  XR_LOG_INFO("Aimer bullet_speed source=%s speed=%.2f m/s prev=%.2f m/s",
              source, new_bullet_speed,
              have_logged_bullet_speed_ ? last_logged_bullet_speed_ : old_bullet_speed);
  last_logged_bullet_speed_ = new_bullet_speed;
  have_logged_bullet_speed_ = true;
}

/**
 * @brief 处理裁判系统摘要反馈。
 */
inline void AimerCore::RefereeSummaryCallback(const AimerRefereeSummary& summary)
{
  UpdateBulletSpeed(summary.launcher_data.bullet_speed, "host/robot_game_ref");
  LogHeatStatus(std::numeric_limits<double>::quiet_NaN(),
                static_cast<double>(summary.robot_status.shooter_heat_limit),
                static_cast<double>(summary.robot_status.shooter_cooling_value),
                "host/robot_game_ref", false);
}

/**
 * @brief 按变化量记录热量、热量上限和冷却值。
 */
inline void AimerCore::LogHeatStatus(double current_heat, double heat_limit,
                                 double cooling, const char* source,
                                 bool force)
{
  if (!cfg_.enable_runtime_log)
  {
    return;
  }

  LibXR::Mutex::LockGuard lock(runtime_log_lock_);
  const bool current_valid = std::isfinite(current_heat);
  const bool heat_limit_valid = std::isfinite(heat_limit);
  const bool cooling_valid = std::isfinite(cooling);
  bool should_log = force || !have_logged_heat_status_;
  if (!should_log && current_valid)
  {
    should_log = std::abs(current_heat - last_logged_heat_) >=
        std::max(0.0, cfg_.heat_log_delta);
  }
  if (!should_log && heat_limit_valid)
  {
    should_log = std::abs(heat_limit - last_logged_heat_limit_) >=
        std::max(0.0, cfg_.heat_log_delta);
  }
  if (!should_log && cooling_valid)
  {
    should_log = std::abs(cooling - last_logged_cooling_) >=
        std::max(0.0, cfg_.heat_log_delta);
  }
  if (!should_log)
  {
    return;
  }

  if (current_valid)
  {
    XR_LOG_INFO("Aimer heat source=%s heat=%.1f limit=%.1f cooling=%.1f bullet=%.2f m/s",
                source, current_heat, heat_limit, cooling,
                bullet_speed_.load(std::memory_order_relaxed));
    last_logged_heat_ = current_heat;
  }
  else
  {
    XR_LOG_INFO("Aimer heat source=%s heat=unknown limit=%.1f cooling=%.1f bullet=%.2f m/s",
                source, heat_limit, cooling,
                bullet_speed_.load(std::memory_order_relaxed));
  }
  last_logged_heat_limit_ = heat_limit;
  last_logged_cooling_ = cooling;
  have_logged_heat_status_ = true;
}

/**
 * @brief 在自动开火状态翻转时输出统计日志。
 */
inline void AimerCore::LogFireState(const ArmorTrackerTarget& target_msg, bool fire,
                                double bullet_speed)
{
  if (!cfg_.enable_runtime_log)
  {
    return;
  }

  LibXR::Mutex::LockGuard lock(runtime_log_lock_);
  if (have_logged_fire_state_ && fire == last_logged_fire_state_)
  {
    return;
  }

  const double current_heat = last_logged_heat_;
  const bool heat_valid = have_logged_heat_status_;
  if (heat_valid)
  {
    XR_LOG_INFO(
        "Aimer fire state=%s target=%d tracking=%d ts=%llu yaw=%.3f pitch=%.3f bullet=%.2f heat=%.1f limit=%.1f cooling=%.1f",
        fire ? "ON" : "OFF", static_cast<int>(target_msg.id),
        target_msg.tracking ? 1 : 0,
        static_cast<unsigned long long>(target_msg.image_timestamp_us),
        gimbal_plan_msg_.yaw, gimbal_plan_msg_.pitch, bullet_speed, current_heat,
        last_logged_heat_limit_, last_logged_cooling_);
  }
  else
  {
    XR_LOG_INFO(
        "Aimer fire state=%s target=%d tracking=%d ts=%llu yaw=%.3f pitch=%.3f bullet=%.2f",
        fire ? "ON" : "OFF", static_cast<int>(target_msg.id),
        target_msg.tracking ? 1 : 0,
        static_cast<unsigned long long>(target_msg.image_timestamp_us),
        gimbal_plan_msg_.yaw, gimbal_plan_msg_.pitch, bullet_speed);
  }
  last_logged_fire_state_ = fire;
  have_logged_fire_state_ = true;
}

/**
 * @brief 根据云台姿态消息更新内部姿态缓存。
 * @param gimbal_rotation_msg 云台姿态四元数。
 */
inline void AimerCore::GimbalRotationCallback(LibXR::Quaternion<float> gimbal_rotation_msg)
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
inline bool AimerCore::ShouldAutoFire(const Eigen::Vector3d& target_xyz,
                                  double selected_view_angle, bool shootable,
                                  double yaw)
{
  const double yaw_threshold =
      AimerDetail::DynamicYawFireThreshold(cfg_, target_xyz, selected_view_angle);

  auto remember_command = [this, yaw]()
  {
    last_command_yaw_ = yaw;
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

  if (!has_last_command_)
  {
    remember_command();
    return false;
  }

  const double command_error_yaw =
      std::abs(AimerDetail::LimitRad(last_command_yaw_ - yaw));
  const double gimbal_error_yaw = std::abs(AimerDetail::LimitRad(gimbal_yaw - yaw));

  const bool command_stable = command_error_yaw < yaw_threshold * 2.0;
  const bool gimbal_aligned = gimbal_error_yaw < yaw_threshold;

  remember_command();
  return command_stable && gimbal_aligned;
}

/**
 * @brief 处理一帧 tracker 目标并发布 host 输出。
 * @param target_msg 当前 tracker 目标消息。
 */
inline void AimerCore::TargetCallback(const ArmorTrackerTarget& target_msg)
{
  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;

  auto publish_outputs = [&](double publish_bullet_speed)
  {
    const bool final_fire = gimbal_plan_msg_.control && gimbal_plan_msg_.fire;
    LogFireState(target_msg, final_fire, publish_bullet_speed);

    AimerHostGimbalTarget host_gimbal{};
    if (gimbal_plan_msg_.control)
    {
      host_gimbal.pit = gimbal_plan_msg_.pitch;
      host_gimbal.yaw = gimbal_plan_msg_.yaw;
      host_gimbal.pit_dot = gimbal_plan_msg_.pitch_vel;
      host_gimbal.yaw_dot = gimbal_plan_msg_.yaw_vel;
      host_gimbal.pit_ddot = gimbal_plan_msg_.pitch_acc;
      host_gimbal.yaw_ddot = gimbal_plan_msg_.yaw_acc;
    }
    AimerHostFireNotify host_fire{final_fire};

    host_gimbal_topic_.Publish(host_gimbal);
    host_fire_topic_.Publish(host_fire);
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
    publish_outputs(bullet_speed);
    return;
  }

  AimerDetail::PredictedTarget base_target{target_msg};
  base_target.Predict(delay_time);
  AimerDetail::AimPoint aim_point =
      AimerDetail::ChooseAimPoint(base_target, lock_id_);
  if (!aim_point.valid)
  {
    ResetGimbalPlanHistory();
    publish_outputs(bullet_speed);
    return;
  }

  const Eigen::Vector3d first_xyz = aim_point.xyza.head<3>();
  const double first_horizontal_distance = AimerDetail::HorizontalDistance(first_xyz);
  const auto first_trajectory = AimerDetail::SolveTrajectoryPitch(
      bullet_speed, first_horizontal_distance, AimerDetail::BallisticHeight(first_xyz));
  if (first_trajectory.unsolvable)
  {
    ResetGimbalPlanHistory();
    publish_outputs(bullet_speed);
    return;
  }

  AimerDetail::PredictedTarget hit_target = base_target;
  hit_target.Predict(first_trajectory.fly_time);
  aim_point = AimerDetail::ChooseAimPoint(hit_target, lock_id_);
  if (!aim_point.valid)
  {
    ResetGimbalPlanHistory();
    publish_outputs(bullet_speed);
    return;
  }

  const Eigen::Vector3d hit_xyz = aim_point.xyza.head<3>();
  const double hit_horizontal_distance = AimerDetail::HorizontalDistance(hit_xyz);
  const auto trajectory = AimerDetail::SolveTrajectoryPitch(
      bullet_speed, hit_horizontal_distance, AimerDetail::BallisticHeight(hit_xyz));
  if (trajectory.unsolvable)
  {
    ResetGimbalPlanHistory();
    publish_outputs(bullet_speed);
    return;
  }

  const Eigen::Vector3d final_xyz = aim_point.xyza.head<3>();
  const double yaw = AimerDetail::LimitRad(
      AimerDetail::BearingYaw(final_xyz) + cfg_.yaw_offset * AimerDetail::DEG2RAD);
  const double pitch = -(trajectory.pitch + cfg_.pitch_offset * AimerDetail::DEG2RAD);

  const bool raw_fire =
      ShouldAutoFire(final_xyz, aim_point.view_angle, aim_point.shootable, yaw);
  BuildGimbalPlan(target_msg, delay_time, true, raw_fire, yaw, pitch, bullet_speed);
  publish_outputs(bullet_speed);
}
