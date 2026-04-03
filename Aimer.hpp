#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: sp_vision style aimer with iterative ballistic solve
constructor_args:
  cfg:
    yaw_offset: -1.0
    pitch_offset: -1.4
    comming_angle: 60.0
    leaving_angle: 20.0
    yaw_rate_threshold: 2.0
    high_speed_delay_time: 0.03
    low_speed_delay_time: 0.015
    default_bullet_speed: 23.0
    min_valid_bullet_speed: 14.0
    first_tolerance: 3.0
    second_tolerance: 2.0
    judge_distance: 2.0
    auto_fire: true
template_args: []
required_hardware: []
depends:
  - qdu-future/ArmorTracker
=== END MANIFEST === */
// clang-format on

#include <Eigen/Dense>

#include <chrono>
#include <cstdint>

#include "ArmorTracker.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "mutex.hpp"

class Aimer : public LibXR::Application
{
 public:
  struct Config
  {
    double yaw_offset{-1.0};
    double pitch_offset{-1.4};
    double comming_angle{60.0};
    double leaving_angle{20.0};
    double yaw_rate_threshold{2.0};
    double high_speed_delay_time{0.03};
    double low_speed_delay_time{0.015};
    double default_bullet_speed{23.0};
    double min_valid_bullet_speed{14.0};
    double first_tolerance{3.0};
    double second_tolerance{2.0};
    double judge_distance{2.0};
    bool auto_fire{true};
  };

  struct AimerMetrics
  {
    uint64_t frame_index{0};
    bool target_tracking{false};
    bool valid{false};
    bool converged{false};
    uint32_t iteration_count{0};
    uint32_t selected_armor_index{0};
    ArmorNumber target_id{ArmorNumber::INVALID};
    double latency_ms{0.0};
    double bullet_speed{0.0};
    double delay_time_s{0.0};
    double fly_time_s{0.0};
    double yaw{0.0};
    double pitch{0.0};
    bool is_fire{false};
  };

  Aimer(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, Config cfg);

  void OnMonitor() override {}

 private:
  void BulletSpeedCallback(float bullet_speed_msg);
  void GimbalRotationCallback(LibXR::Quaternion<float> gimbal_rotation_msg);
  void TargetCallback(SolveTrajectory::Target& target_msg);
  bool ShouldAutoFire(const Eigen::Vector3d& target_xyz, double yaw);

 private:
  Config cfg_{};
  double bullet_speed_{23.0};
  int lock_id_{-1};
  ArmorNumber last_target_id_{ArmorNumber::INVALID};
  uint64_t frame_index_{0};
  bool has_last_command_{false};
  double last_command_yaw_{0.0};
  bool has_gimbal_rotation_{false};
  double last_fire_tolerance_rad_{0.0};
  double last_fire_command_error_rad_{0.0};
  double last_fire_gimbal_error_rad_{0.0};
  double last_fire_gimbal_yaw_rad_{0.0};
  LibXR::Quaternion<double> gimbal_rotation_{1.0, 0.0, 0.0, 0.0};
  mutable LibXR::Mutex gimbal_rotation_lock_{};

  AimerMetrics metrics_msg_{};
  LibXR::EulerAngle<float> target_euler_msg_{};
  ArmorTracker::Send send_msg_{};

  LibXR::Topic::Domain aimer_domain_ = LibXR::Topic::Domain("aimer");
  LibXR::Topic metrics_topic_ =
      LibXR::Topic("metrics", sizeof(AimerMetrics), &aimer_domain_);

  LibXR::Topic::Domain tracker_domain_ = LibXR::Topic::Domain("tracker");
  LibXR::Topic target_euler_topic_ =
      LibXR::Topic("target_eulr", sizeof(LibXR::EulerAngle<float>), &tracker_domain_);
  LibXR::Topic send_topic_ =
      LibXR::Topic("send", sizeof(ArmorTracker::Send), &tracker_domain_);
};
