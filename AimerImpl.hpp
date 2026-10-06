#pragma once

/**
 * @file AimerImpl.hpp
 * @brief Aimer 模块的内联运行时实现。
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "AimerPlanner.hpp"

/**
 * @brief 构造 Aimer 运行核心并注册 referee、gimbal 输入 topic 回调。
 */
inline AimerCore::AimerCore(Config cfg)
    : cfg_(std::move(cfg)),
      referee_topic_name_(cfg_.referee_topic),
      bullet_speed_(cfg_.default_bullet_speed)
{
  cfg_.referee_topic = referee_topic_name_.View();
  lead_calibrator_ = AimerDetail::LeadCalibrator(
      {.calibration_batches = cfg_.lead_calibration_batches,
       .max_adjust_s = cfg_.lead_calibration_max_adjust_s,
       .monitor_threshold_s = cfg_.lead_calibration_monitor_threshold_s,
       .monitor_batches = cfg_.lead_calibration_monitor_batches});
  SetupGimbalPlanSolvers();
  RegisterHostInputCallbacks();
}

/**
 * @brief 设置内置 preview 的状态接收器。
 */
inline void AimerCore::SetPreviewSink(PreviewSink sink, void* context)
{
  preview_sink_ = sink;
  preview_context_ = context;
}

/**
 * @brief 提交本帧 Aimer 状态给内置 preview。
 */
inline void AimerCore::PublishPreviewState(const AimerPreviewFrame& state)
{
  if (preview_sink_ != nullptr)
  {
    preview_sink_(preview_context_, state);
  }
}

/**
 * @brief 注册裁判系统与云台姿态输入回调。
 */
inline void AimerCore::RegisterHostInputCallbacks() { RegisterRefereeSummaryInput(); }

inline void AimerCore::UpdateGimbalRotationFromSyncedImu(
    const std::array<float, 4>& rotation_wxyz)
{
  LibXR::Mutex::LockGuard lock(gimbal_rotation_lock_);
  gimbal_rotation_ = LibXR::Quaternion<double>(
      static_cast<double>(rotation_wxyz[0]), static_cast<double>(rotation_wxyz[1]),
      static_cast<double>(rotation_wxyz[2]), static_cast<double>(rotation_wxyz[3]));
  has_gimbal_rotation_ = true;
}

inline void AimerCore::ClearGimbalRotation()
{
  LibXR::Mutex::LockGuard lock(gimbal_rotation_lock_);
  has_gimbal_rotation_ = false;
  gimbal_rotation_ = LibXR::Quaternion<double>(1.0, 0.0, 0.0, 0.0);
}

/**
 * @brief 注册 C 板回传的云台姿态四元数输入 topic。
 */
inline void AimerCore::RegisterGimbalQuatInput()
{
  LibXR::Topic topic = LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>(
      "ahrs_quaternion", &host_domain_);
  auto callback = LibXR::Topic::Callback::Create(
      [](bool, AimerCore* self, const LibXR::Quaternion<float>& rotation)
      { self->GimbalRotationCallback(rotation); }, this);
  topic.RegisterCallback(callback);
}

/**
 * @brief 注册裁判系统摘要输入 topic。
 */
inline void AimerCore::RegisterRefereeSummaryInput()
{
  LibXR::Topic referee_topic = LibXR::Topic::FindOrCreate<AimerRefereeSummary>(
      referee_topic_name_.CStr(), &host_domain_);
  auto referee_callback = LibXR::Topic::Callback::Create(
      [](bool, AimerCore* self, const AimerRefereeSummary& summary)
      { self->RefereeSummaryCallback(summary); }, this);
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
  const bool should_log = !have_logged_bullet_speed_ ||
                          std::abs(new_bullet_speed - last_logged_bullet_speed_) >=
                              std::max(0.0, cfg_.bullet_speed_log_delta);
  if (!should_log)
  {
    return;
  }

  XR_LOG_INFO("Aimer bullet_speed source=%s speed=%.2f m/s prev=%.2f m/s", source,
              new_bullet_speed,
              have_logged_bullet_speed_ ? last_logged_bullet_speed_ : old_bullet_speed);
  last_logged_bullet_speed_ = new_bullet_speed;
  have_logged_bullet_speed_ = true;
}

/**
 * @brief 处理裁判系统摘要反馈。
 */
inline void AimerCore::RefereeSummaryCallback(const AimerRefereeSummary& summary)
{
  referee_heat_limit_.store(static_cast<double>(summary.robot_status.shooter_heat_limit),
                            std::memory_order_relaxed);
  referee_cooling_.store(static_cast<double>(summary.robot_status.shooter_cooling_value),
                         std::memory_order_relaxed);
  UpdateBulletSpeed(cfg_.default_bullet_speed, referee_topic_name_.CStr());
  LogHeatStatus(std::numeric_limits<double>::quiet_NaN(),
                static_cast<double>(summary.robot_status.shooter_heat_limit),
                static_cast<double>(summary.robot_status.shooter_cooling_value),
                referee_topic_name_.CStr(), false);
}

/**
 * @brief 按变化量记录热量、热量上限和冷却值。
 */
inline void AimerCore::LogHeatStatus(double current_heat, double heat_limit,
                                     double cooling, const char* source, bool force)
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
    should_log =
        std::abs(current_heat - last_logged_heat_) >= std::max(0.0, cfg_.heat_log_delta);
  }
  if (!should_log && heat_limit_valid)
  {
    should_log = std::abs(heat_limit - last_logged_heat_limit_) >=
                 std::max(0.0, cfg_.heat_log_delta);
  }
  if (!should_log && cooling_valid)
  {
    should_log =
        std::abs(cooling - last_logged_cooling_) >= std::max(0.0, cfg_.heat_log_delta);
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
    have_logged_current_heat_ = true;
  }
  else
  {
    XR_LOG_INFO(
        "Aimer heat source=%s heat=unknown limit=%.1f cooling=%.1f bullet=%.2f m/s",
        source, heat_limit, cooling, bullet_speed_.load(std::memory_order_relaxed));
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

  if (have_logged_current_heat_)
  {
    XR_LOG_INFO(
        "Aimer fire state=%s target=%d tracking=%d ts=%llu yaw=%.3f roll=%.3f "
        "bullet=%.2f heat=%.1f limit=%.1f cooling=%.1f",
        fire ? "ON" : "OFF", static_cast<int>(target_msg.id), target_msg.tracking ? 1 : 0,
        static_cast<unsigned long long>(target_msg.image_timestamp_us),
        gimbal_plan_msg_.yaw, gimbal_plan_msg_.roll, bullet_speed, last_logged_heat_,
        last_logged_heat_limit_, last_logged_cooling_);
  }
  else if (have_logged_heat_status_)
  {
    XR_LOG_INFO(
        "Aimer fire state=%s target=%d tracking=%d ts=%llu yaw=%.3f roll=%.3f "
        "bullet=%.2f heat=unknown limit=%.1f cooling=%.1f",
        fire ? "ON" : "OFF", static_cast<int>(target_msg.id), target_msg.tracking ? 1 : 0,
        static_cast<unsigned long long>(target_msg.image_timestamp_us),
        gimbal_plan_msg_.yaw, gimbal_plan_msg_.roll, bullet_speed,
        last_logged_heat_limit_, last_logged_cooling_);
  }
  else
  {
    XR_LOG_INFO(
        "Aimer fire state=%s target=%d tracking=%d ts=%llu yaw=%.3f roll=%.3f "
        "bullet=%.2f",
        fire ? "ON" : "OFF", static_cast<int>(target_msg.id), target_msg.tracking ? 1 : 0,
        static_cast<unsigned long long>(target_msg.image_timestamp_us),
        gimbal_plan_msg_.yaw, gimbal_plan_msg_.roll, bullet_speed);
  }
  last_logged_fire_state_ = fire;
  have_logged_fire_state_ = true;
}

/**
 * @brief 根据云台姿态消息更新内部姿态缓存。
 * @param gimbal_rotation_msg 云台姿态四元数。
 */
inline void AimerCore::GimbalRotationCallback(
    LibXR::Quaternion<float> gimbal_rotation_msg)
{
  LibXR::Mutex::LockGuard lock(gimbal_rotation_lock_);
  if (cfg_.convert_raw_gimbal_quat_to_body)
  {
    gimbal_rotation_ = LibXR::Quaternion<double>(
        gimbal_rotation_msg.w(), -static_cast<double>(gimbal_rotation_msg.y()),
        static_cast<double>(gimbal_rotation_msg.x()),
        static_cast<double>(gimbal_rotation_msg.z()));
  }
  else
  {
    gimbal_rotation_ =
        LibXR::Quaternion<double>(gimbal_rotation_msg.w(), gimbal_rotation_msg.x(),
                                  gimbal_rotation_msg.y(), gimbal_rotation_msg.z());
  }
  has_gimbal_rotation_ = true;
}

/**
 * @brief 判断当前计划命令是否满足自动开火条件。
 * @param shot_candidate 当前发射请求对应的未来命中候选。
 * @param plan_fire_enabled 命中面和当前云台计划是否允许开火。
 * @param yaw 命令 yaw，单位 rad。
 * @param roll 命令机械 roll 轴，单位 rad。
 * @return 所有开火门控通过时返回 true。
 */
inline bool AimerCore::ShouldAutoFire(const AimerShotCandidate& shot_candidate,
                                      bool plan_fire_enabled, double yaw, double roll)
{
  auto remember_command = [this, yaw, roll]()
  {
    last_command_yaw_ = yaw;
    last_command_roll_ = roll;
    last_command_yaw_vel_ = static_cast<double>(gimbal_plan_msg_.yaw_vel);
    last_command_roll_vel_ = static_cast<double>(gimbal_plan_msg_.roll_vel);
    last_command_image_us_ = current_image_us_;
    has_last_command_ = true;
  };

  if (!shot_candidate.valid)
  {
    remember_command();
    return false;
  }

  const Eigen::Vector3d target_xyz = shot_candidate.hit_xyza.head<3>();
  const double yaw_threshold =
      AimerDetail::DynamicYawFireThreshold(cfg_, target_xyz, shot_candidate.view_angle);
  const double roll_threshold = AimerDetail::DynamicRollFireThreshold(cfg_, target_xyz);

  if (!cfg_.auto_fire || !plan_fire_enabled || !shot_candidate.face_shootable_at_hit)
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
  const double gimbal_roll = gimbal_euler[0];
  const double gimbal_yaw = gimbal_euler[2];

  if (!has_last_command_)
  {
    remember_command();
    return false;
  }

  // 上一帧命令按其角速度推进到本帧再比较，匀速转动的命令不算不稳定。
  // The previous command is advanced to this frame with its rate before the comparison,
  // so a command that turns at a steady rate counts as stable.
  const double command_dt =
      current_image_us_ > last_command_image_us_
          ? static_cast<double>(current_image_us_ - last_command_image_us_) * 1e-6
          : 0.0;
  const double command_error_yaw = std::abs(AimerDetail::LimitRad(
      last_command_yaw_ + last_command_yaw_vel_ * command_dt - yaw));
  const double command_error_roll =
      std::abs(last_command_roll_ + last_command_roll_vel_ * command_dt - roll);
  const double gimbal_error_yaw_signed = AimerDetail::LimitRad(gimbal_yaw - yaw);
  const double gimbal_error_yaw = std::abs(gimbal_error_yaw_signed);
  const double gimbal_error_roll = std::abs(AimerDetail::LimitRad(gimbal_roll - roll));

  const bool command_stable = command_error_yaw < yaw_threshold * 2.0 &&
                              command_error_roll < roll_threshold * 2.0;
  const bool gimbal_aligned =
      gimbal_error_yaw < yaw_threshold && gimbal_error_roll < roll_threshold;

  bool fire = command_stable && gimbal_aligned;
  if (cfg_.heat_aware_fire)
  {
    fire = HeatAwareFire(shot_candidate, gimbal_error_yaw_signed,
                         static_cast<double>(gimbal_plan_msg_.yaw_vel), fire);
  }
  remember_command();
  return fire;
}

/**
 * @brief 在已有开火门控之后应用按热量分配的开火判定。
 *
 * 热量按裁判系统的热量上限和冷却值在本地推算：每次计入的开火加单发热量，按冷却值连续
 * 衰减。命中概率按出膛时刻估计：横向偏差由请求时刻云台误差留到出膛的部分和随指令角速度
 * 增长的偏差组成，标准差由基础项、随预测时域增长的相位项和随指令角速度增长的项组成。
 */
inline bool AimerCore::HeatAwareFire(const AimerShotCandidate& shot_candidate,
                                     double gimbal_error_yaw, double command_yaw_rate,
                                     bool gates_passed)
{
  const double limit = referee_heat_limit_.load(std::memory_order_relaxed);
  const double cooling = referee_cooling_.load(std::memory_order_relaxed);
  if (!(limit > 0.0))
  {
    return gates_passed;
  }

  const uint64_t now_us = current_image_us_;
  AimerDetail::CoolHeat(heat_fire_state_, now_us, cooling);

  const Eigen::Vector3d target_xyz = shot_candidate.hit_xyza.head<3>();
  const double distance = std::max(0.3, AimerDetail::HorizontalDistance(target_xyz));
  const double half_width =
      0.5 * AimerDetail::SMALL_ARMOR_WIDTH_M * std::cos(std::abs(shot_candidate.view_angle)) -
      AimerDetail::FIRE_BULLET_SPREAD_M;
  const double horizon = std::max(0.0, shot_candidate.fly_time) + cfg_.heat_fire_horizon_extra_s;
  const double phase_sigma = cfg_.heat_fire_phase_sigma_rad_s * horizon * current_target_radius_;
  const double bias = AimerDetail::ExitLateralBias(distance, gimbal_error_yaw,
                                                   cfg_.heat_fire_gimbal_error_gain,
                                                   command_yaw_rate, cfg_.heat_fire_rate_bias_s);
  const double sigma =
      AimerDetail::ExitLateralSigma(cfg_.heat_fire_sigma_m, phase_sigma, distance,
                                    command_yaw_rate, cfg_.heat_fire_rate_spread_s);
  const double p_hit = AimerDetail::HitProbability(half_width, bias, sigma);

  const double since_shot_s =
      static_cast<double>(now_us - std::min(now_us, heat_fire_state_.relax_ref_us)) * 1e-6;
  const double threshold = AimerDetail::HeatFireThreshold(
      cfg_.heat_fire_p_low, cfg_.heat_fire_p_high, cfg_.heat_fire_p_floor,
      cfg_.heat_fire_relax_s, heat_fire_state_.heat / limit, since_shot_s);

  const bool fire = gates_passed && heat_fire_state_.heat + cfg_.heat_fire_shot_heat <= limit &&
                    p_hit >= threshold;
  if (fire)
  {
    AimerDetail::CountShot(heat_fire_state_, now_us, cfg_.heat_fire_shot_heat,
                           cfg_.heat_fire_min_interval_s);
  }
  return fire;
}

/**
 * @brief 输出 tracker 目标回调的累计耗时统计。
 */
inline void AimerCore::OnMonitor()
{
  const auto summary = target_callback_duration_.GetSummary();
  XR_LOG_INFO(
      "Aimer monitor: target_callback count=%llu average_us=%llu "
      "minimum_us=%llu maximum_us=%llu",
      static_cast<unsigned long long>(summary.sample_count),
      static_cast<unsigned long long>(summary.average_us),
      static_cast<unsigned long long>(summary.minimum_us),
      static_cast<unsigned long long>(summary.maximum_us));
}

/**
 * @brief 处理一帧 tracker 目标并发布 host 输出。
 * @param target_msg 当前 tracker 目标消息。
 */
inline void AimerCore::TargetCallback(const ArmorTrackerTarget& target_msg)
{
  auto target_callback_measurement = target_callback_duration_.Measure();
  if (cfg_.lead_calibration)
  {
    UpdateLeadCalibration(target_msg, gimbal_plan_msg_.control,
                          static_cast<double>(gimbal_plan_msg_.yaw_vel));
  }
  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  current_image_us_ = target_msg.image_timestamp_us;
  current_target_radius_ = target_msg.radius_1 > 0.05 ? target_msg.radius_1 : 0.2;
  if (target_msg.tracking && (!heat_fire_tracking_ || target_msg.id != last_target_id_))
  {
    AimerDetail::StartEngagement(heat_fire_state_, current_image_us_);
  }
  heat_fire_tracking_ = target_msg.tracking;
  AimerPreviewFrame preview_frame{};
  preview_frame.image_timestamp_us = target_msg.image_timestamp_us;
  preview_frame.have_target = true;
  preview_frame.target = target_msg;

  auto publish_outputs = [&](double publish_bullet_speed)
  {
    const bool final_fire = gimbal_plan_msg_.control && gimbal_plan_msg_.fire;
    LogFireState(target_msg, final_fire, publish_bullet_speed);

    AimerHostGimbalTarget host_gimbal{};
    if (gimbal_plan_msg_.control)
    {
      host_gimbal.rol = gimbal_plan_msg_.roll;
      host_gimbal.pit = gimbal_plan_msg_.roll;
      host_gimbal.yaw = gimbal_plan_msg_.yaw;
      host_gimbal.rol_dot = gimbal_plan_msg_.roll_vel;
      host_gimbal.pit_dot = gimbal_plan_msg_.roll_vel;
      host_gimbal.yaw_dot = gimbal_plan_msg_.yaw_vel;
      host_gimbal.rol_ddot = gimbal_plan_msg_.roll_acc;
      host_gimbal.pit_ddot = gimbal_plan_msg_.roll_acc;
      host_gimbal.yaw_ddot = gimbal_plan_msg_.yaw_acc;
    }
    AimerHostFireNotify host_fire{final_fire};

    host_gimbal_topic_.Publish(host_gimbal);
    host_fire_topic_.Publish(host_fire);
    preview_frame.have_host_fire = true;
    preview_frame.host_fire = host_fire;
    PublishPreviewState(preview_frame);
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

  const double lead_adjust = cfg_.lead_calibration ? lead_calibrator_.Adjust() : 0.0;
  const double delay_time =
      target_msg.tracking ? AimerDetail::FixedPredictDelay(cfg_, target_msg) + lead_adjust
                          : 0.0;
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
      AimerDetail::ChooseAimPoint(cfg_, base_target, lock_id_);
  if (!aim_point.valid)
  {
    ResetGimbalPlanHistory();
    publish_outputs(bullet_speed);
    return;
  }

  const Eigen::Vector3d first_xyz = aim_point.xyza.head<3>();
  const double first_horizontal_distance = AimerDetail::HorizontalDistance(first_xyz);
  const auto first_trajectory = AimerDetail::SolveTrajectoryElevation(
      bullet_speed, first_horizontal_distance, AimerDetail::BallisticHeight(first_xyz),
      cfg_.ballistic_drag_k, cfg_.ballistic_integration_dt_s,
      cfg_.ballistic_max_iterations, cfg_.ballistic_min_elevation_deg,
      cfg_.ballistic_max_elevation_deg);
  if (first_trajectory.unsolvable)
  {
    ResetGimbalPlanHistory();
    publish_outputs(bullet_speed);
    return;
  }

  AimerDetail::PredictedTarget hit_target = base_target;
  hit_target.Predict(first_trajectory.fly_time);
  aim_point = AimerDetail::ChooseAimPoint(cfg_, hit_target, lock_id_);
  if (!aim_point.valid)
  {
    ResetGimbalPlanHistory();
    publish_outputs(bullet_speed);
    return;
  }

  const Eigen::Vector3d hit_xyz = aim_point.xyza.head<3>();
  const double hit_horizontal_distance = AimerDetail::HorizontalDistance(hit_xyz);
  const auto trajectory = AimerDetail::SolveTrajectoryElevation(
      bullet_speed, hit_horizontal_distance, AimerDetail::BallisticHeight(hit_xyz),
      cfg_.ballistic_drag_k, cfg_.ballistic_integration_dt_s,
      cfg_.ballistic_max_iterations, cfg_.ballistic_min_elevation_deg,
      cfg_.ballistic_max_elevation_deg);
  if (trajectory.unsolvable)
  {
    ResetGimbalPlanHistory();
    publish_outputs(bullet_speed);
    return;
  }

  const Eigen::Vector3d final_xyz = aim_point.xyza.head<3>();
  preview_frame.aim_point_valid = true;
  preview_frame.aim_point = final_xyz;
  preview_frame.aim_armor_index = aim_point.armor_index;
  preview_frame.aim_xyza = aim_point.xyza;
  const double yaw = AimerDetail::LimitRad(AimerDetail::BearingYaw(final_xyz) +
                                           cfg_.yaw_offset * AimerDetail::DEG2RAD);
  const double roll = trajectory.elevation + cfg_.roll_offset * AimerDetail::DEG2RAD;

  const AimerShotCandidate direct_shot_candidate =
      AimerDetail::MakeShotCandidate(aim_point, yaw, roll, trajectory.fly_time);

  AimerShotCandidate fire_shot_candidate = direct_shot_candidate;
  BuildGimbalPlan(target_msg, delay_time, true, yaw, roll, bullet_speed,
                  direct_shot_candidate, fire_shot_candidate);

  gimbal_plan_msg_.fire = ShouldAutoFire(fire_shot_candidate, gimbal_plan_msg_.fire,
                                         gimbal_plan_msg_.yaw, gimbal_plan_msg_.roll);
  publish_outputs(bullet_speed);
}

/**
 * @brief 在线标定指向超前量。
 *
 * 本帧图像时刻的云台 yaw 和当前生效指令的 yaw 角速度存为样本，到达时刻取本帧图像时刻加
 * 飞行时间。到达时刻不晚于本帧图像时刻的样本用本帧目标状态回推到到达时刻，取方位最接近
 * 云台 yaw 的装甲板求残差；本帧比到达时刻晚 25 ms 以上、视角超过 70 度或残差超过 3 度的
 * 样本不用。
 */
inline void AimerCore::UpdateLeadCalibration(const ArmorTrackerTarget& target_msg,
                                             bool command_in_force, double command_yaw_rate)
{
  constexpr uint64_t MAX_OBSERVATION_LATE_US = 25000;
  constexpr double MAX_VIEW_RAD = 70.0 * AimerDetail::DEG2RAD;
  constexpr double MAX_RESIDUAL_RAD = 3.0 * AimerDetail::DEG2RAD;

  if (!target_msg.tracking || target_msg.id != last_target_id_)
  {
    lead_pending_.clear();
    return;
  }
  const uint64_t now_us = target_msg.image_timestamp_us;

  while (!lead_pending_.empty() && lead_pending_.front().arrival_us <= now_us)
  {
    const LeadSample sample = lead_pending_.front();
    lead_pending_.pop_front();
    if (now_us - sample.arrival_us > MAX_OBSERVATION_LATE_US)
    {
      continue;
    }
    AimerDetail::PredictedTarget observed{target_msg};
    observed.Predict(-static_cast<double>(now_us - sample.arrival_us) * 1e-6);
    double residual = std::numeric_limits<double>::infinity();
    double view = std::numeric_limits<double>::infinity();
    for (const auto& xyza : observed.GetArmorXYZAList())
    {
      const double candidate =
          AimerDetail::LimitRad(sample.gimbal_yaw - AimerDetail::BearingYaw(xyza.head<3>()));
      if (std::abs(candidate) < std::abs(residual))
      {
        residual = candidate;
        view = AimerDetail::ViewAngle(observed.msg, xyza);
      }
    }
    if (std::abs(view) <= MAX_VIEW_RAD && std::abs(residual) < MAX_RESIDUAL_RAD)
    {
      lead_calibrator_.AddSample(sample.command_yaw_rate, residual);
    }
  }

  bool has_gimbal_rotation = false;
  double gimbal_yaw = 0.0;
  {
    LibXR::Mutex::LockGuard lock(gimbal_rotation_lock_);
    has_gimbal_rotation = has_gimbal_rotation_;
    gimbal_yaw = gimbal_rotation_.ToEulerAngleZYX()[2];
  }
  if (command_in_force && has_gimbal_rotation && std::isfinite(command_yaw_rate))
  {
    double bullet_speed = bullet_speed_.load(std::memory_order_relaxed);
    if (std::isnan(bullet_speed) || bullet_speed < cfg_.min_valid_bullet_speed)
    {
      bullet_speed = cfg_.default_bullet_speed;
    }
    // 水平距离除以弹速再乘 1.05，近似计入阻力；只用于确定用哪一帧检查。
    // Horizontal distance over bullet speed times 1.05 approximates the drag; it only
    // selects the frame used for the check.
    const double fly_time =
        AimerDetail::HorizontalDistance(target_msg.position) / bullet_speed * 1.05;
    lead_pending_.push_back({now_us + static_cast<uint64_t>(std::llround(fly_time * 1e6)),
                             gimbal_yaw, command_yaw_rate});
  }

  if (!lead_calibrator_.BatchReady())
  {
    return;
  }
  using Event = AimerDetail::LeadCalibrator::Event;
  const auto result = lead_calibrator_.CloseBatch();
  if (result.event == Event::CORRECTED || result.event == Event::FROZEN ||
      result.event == Event::RESTARTED)
  {
    lead_pending_.clear();
  }
  if (!cfg_.enable_runtime_log)
  {
    return;
  }
  switch (result.event)
  {
    case Event::SKIPPED:
      XR_LOG_INFO("Aimer lead calibration: batch skipped, too little command yaw rate");
      break;
    case Event::CORRECTED:
      XR_LOG_INFO("Aimer lead calibration: lead=%.1f ms adjust=%.1f ms samples=%.0f",
                  result.lead_s * 1e3, result.adjust_s * 1e3, result.samples);
      break;
    case Event::FROZEN:
      XR_LOG_INFO(
          "Aimer lead calibration: frozen adjust=%.1f ms; to keep it, add %.4f s to "
          "gimbal_response_delay_s",
          result.adjust_s * 1e3, result.adjust_s);
      break;
    case Event::MONITORED:
      XR_LOG_INFO("Aimer lead calibration: monitor lead=%.1f ms adjust=%.1f ms",
                  result.lead_s * 1e3, result.adjust_s * 1e3);
      break;
    case Event::RESTARTED:
      XR_LOG_INFO("Aimer lead calibration: lead %.1f ms persists, recalibrating",
                  result.lead_s * 1e3);
      break;
    case Event::NONE:
      break;
  }
}
