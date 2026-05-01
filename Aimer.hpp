#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: sp_vision style aimer with iterative ballistic solve and TinyMPC gimbal plan output
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
    enable_mpc_plan: true
    mpc_fire_thresh: 0.0035
    max_yaw_acc: 50.0
    q_yaw_pos: 9000000.0
    q_yaw_vel: 0.0
    r_yaw_acc: 1.0
    max_pitch_acc: 100.0
    q_pitch_pos: 9000000.0
    q_pitch_vel: 0.0
    r_pitch_acc: 1.0
    mpc_max_iter: 7
template_args: []
required_hardware: []
depends:
  - qdu-future/ArmorTracker
=== END MANIFEST === */
// clang-format on

#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <cstdint>

#include "ArmorTracker.hpp"
#include "GimbalPlan.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "mutex.hpp"
#include "tinympc/tiny_api.hpp"

class Aimer : public LibXR::Application
{
 public:
  struct AimerTrajectory
  {
    static constexpr uint8_t MAX_POINTS = 32;

    uint64_t image_timestamp_us{0};
    bool valid{false};
    bool fire{false};
    bool converged{false};
    uint8_t point_count{0};
    uint8_t selected_armor_index{0};
    ArmorNumber target_id{ArmorNumber::INVALID};
    double bullet_speed{0.0};
    double delay_time_s{0.0};
    double fly_time_s{0.0};
    double yaw{0.0};
    double pitch{0.0};
    LibXR::Position<double> aim_point{};
    std::array<LibXR::Position<double>, MAX_POINTS> points{};
  };

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
    bool enable_mpc_plan{true};
    double mpc_fire_thresh{0.0035};
    double max_yaw_acc{50.0};
    double q_yaw_pos{9.0e6};
    double q_yaw_vel{0.0};
    double r_yaw_acc{1.0};
    double max_pitch_acc{100.0};
    double q_pitch_pos{9.0e6};
    double q_pitch_vel{0.0};
    double r_pitch_acc{1.0};
    int mpc_max_iter{7};
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
  void TargetCallback(const SolveTrajectory::Target& target_msg);
  bool ShouldAutoFire(const Eigen::Vector3d& target_xyz, double yaw);
  void BuildTrajectoryMessage(const SolveTrajectory::Target& target_msg,
                              const Eigen::Vector3d& aim_point, double fly_time,
                              double launch_pitch, double bullet_speed, double yaw,
                              double pitch);
  void SetupGimbalPlanSolvers();
  bool BuildMpcGimbalPlan(const SolveTrajectory::Target& target_msg,
                          double bullet_speed, bool fire);
  void BuildFiniteDifferenceGimbalPlan(const SolveTrajectory::Target& target_msg,
                                       bool control, bool fire, double yaw,
                                       double pitch);
  void BuildGimbalPlan(const SolveTrajectory::Target& target_msg, bool control,
                       bool fire, double yaw, double pitch, double bullet_speed);
  void ResetGimbalPlanHistory();

 private:
  Config cfg_{};
  std::atomic<double> bullet_speed_{23.0};
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
  bool has_last_plan_command_{false};
  bool has_last_plan_velocity_{false};
  uint64_t last_plan_timestamp_us_{0};
  double last_plan_yaw_{0.0};
  double last_plan_pitch_{0.0};
  double last_plan_yaw_vel_{0.0};
  double last_plan_pitch_vel_{0.0};
  bool planner_ready_{false};
  bool last_plan_mpc_{false};
  TinySolver* yaw_solver_{nullptr};
  TinySolver* pitch_solver_{nullptr};
  mutable LibXR::Mutex gimbal_rotation_lock_{};

  AimerMetrics metrics_msg_{};
  AimerTrajectory trajectory_msg_{};
  LibXR::EulerAngle<float> target_euler_msg_{};
  ArmorTrackerSend send_msg_{};
  GimbalPlan gimbal_plan_msg_{};

  LibXR::Topic::Domain aimer_domain_ = LibXR::Topic::Domain("aimer");
  LibXR::Topic metrics_topic_ =
      LibXR::Topic("metrics", sizeof(AimerMetrics), &aimer_domain_);
  LibXR::Topic trajectory_topic_ = LibXR::Topic(
      LibXR::Topic::FindOrCreate<AimerTrajectory>("trajectory", &aimer_domain_));

  LibXR::Topic::Domain tracker_domain_ = LibXR::Topic::Domain("tracker");
  LibXR::Topic target_euler_topic_ =
      LibXR::Topic("target_eulr", sizeof(LibXR::EulerAngle<float>), &tracker_domain_);
  LibXR::Topic fire_notify_topic_ =
      LibXR::Topic("fire_notify", sizeof(uint8_t), &tracker_domain_);
  LibXR::Topic send_topic_ =
      LibXR::Topic("send", sizeof(ArmorTrackerSend), &tracker_domain_);
  LibXR::Topic gimbal_plan_topic_ =
      LibXR::Topic("gimbal_plan", sizeof(GimbalPlan), &tracker_domain_);
};
