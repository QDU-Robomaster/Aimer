#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: ballistic aimer with TinyMPC gimbal plan output
constructor_args:
  cfg:
    yaw_offset: -1.0
    pitch_offset: -1.4
    yaw_rate_threshold: 2.0
    default_bullet_speed: 23.0
    min_valid_bullet_speed: 14.0
    auto_fire: true
    image_to_now_s: 0.0
    vision_to_command_delay_s: 0.0
    command_transport_delay_s: 0.0
    gimbal_response_delay_s: 0.0
    fire_delay_s: 0.0
    low_speed_extra_predict_s: 0.015
    high_speed_extra_predict_s: 0.03
    armor_width_m: 0.135
    armor_height_m: 0.055
    bullet_spread_m: 0.015
    min_fire_threshold: 0.003
    max_fire_threshold: 0.05
    enable_pitch_fire_gate: false
    enable_mpc_plan: true
    mpc_fire_thresh: 0.05
    max_yaw_acc: 100.0
    q_yaw_pos: 50.0
    q_yaw_vel: 1.0
    r_yaw_acc: 1.0
    max_pitch_acc: 100.0
    q_pitch_pos: 50.0
    q_pitch_vel: 1.0
    r_pitch_acc: 1.0
    mpc_max_iter: 10
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
#include <fstream>
#include <string>

#include "ArmorTracker.hpp"
#include "ArmorTrackerTarget.hpp"
#include "GimbalPlan.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "mutex.hpp"
#include "tinympc/tiny_api.hpp"

struct AimerSend
{
  bool is_fire{};
  LibXR::Position<double> position{};
  double v_yaw{};
  double pitch{};
  double yaw{};
  Eigen::Matrix<double, 3, 1> cmd_vel_linear = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1> cmd_vel_angular = Eigen::Matrix<double, 3, 1>::Zero();
};

class Aimer : public LibXR::Application
{
 public:
  enum class Strategy : uint8_t
  {
    LOST = 0,
    LOW_SPEED = 1,
    MEDIUM_SPIN = 2,
    OUTPOST = 3,
  };

  enum class SelectReason : uint8_t
  {
    NONE = 0,
    NEAREST_FRONT = 1,
  };

  enum class SwitchReason : uint8_t
  {
    NONE = 0,
    NEW_TARGET = 1,
    NEAREST_CHANGED = 2,
  };

  enum class FireReason : uint8_t
  {
    DISABLED = 0,
    OK = 1,
    NO_TRACK = 2,
    NO_GIMBAL = 3,
    COMMAND_UNSTABLE = 4,
    GIMBAL_NOT_ALIGNED = 5,
    NOT_SHOOTABLE = 6,
    BALLISTIC_UNSOLVABLE = 7,
  };

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
    double yaw_rate_threshold{2.0};
    double default_bullet_speed{23.0};
    double min_valid_bullet_speed{14.0};
    bool auto_fire{true};
    double image_to_now_s{0.0};
    double vision_to_command_delay_s{0.0};
    double command_transport_delay_s{0.0};
    double gimbal_response_delay_s{0.0};
    double fire_delay_s{0.0};
    double low_speed_extra_predict_s{0.015};
    double high_speed_extra_predict_s{0.03};
    double armor_width_m{0.135};
    double armor_height_m{0.055};
    double bullet_spread_m{0.015};
    double min_fire_threshold{0.003};
    double max_fire_threshold{0.05};
    bool enable_pitch_fire_gate{false};
    bool enable_mpc_plan{true};
    double mpc_fire_thresh{0.05};
    double max_yaw_acc{100.0};
    double q_yaw_pos{50.0};
    double q_yaw_vel{1.0};
    double r_yaw_acc{1.0};
    double max_pitch_acc{100.0};
    double q_pitch_pos{50.0};
    double q_pitch_vel{1.0};
    double r_pitch_acc{1.0};
    int mpc_max_iter{10};
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
    double total_hit_delay_s{0.0};
    double yaw{0.0};
    double pitch{0.0};
    double fire_thres_yaw{0.0};
    double fire_thres_pitch{0.0};
    double command_error_yaw{0.0};
    double command_error_pitch{0.0};
    double gimbal_error_yaw{0.0};
    double gimbal_error_pitch{0.0};
    Strategy strategy{Strategy::LOST};
    SelectReason selected_reason{SelectReason::NONE};
    SwitchReason switch_reason{SwitchReason::NONE};
    FireReason fire_reason{FireReason::NO_TRACK};
    bool planner_mpc{false};
    bool is_fire{false};
  };

  struct AimerDecision
  {
    uint64_t frame_id{0};
    uint64_t image_timestamp_us{0};
    uint64_t aimer_receive_time_us{0};
    uint64_t predict_time_us{0};
    uint64_t expected_hit_time_us{0};
    bool target_tracking{false};
    bool valid{false};
    bool converged{false};
    uint8_t candidate_count{0};
    uint8_t selected_armor_index{0};
    ArmorNumber target_id{ArmorNumber::INVALID};
    Strategy strategy{Strategy::LOST};
    SelectReason selected_reason{SelectReason::NONE};
    SwitchReason switch_reason{SwitchReason::NONE};
    FireReason fire_reason{FireReason::NO_TRACK};
    double fixed_delay_s{0.0};
    double fire_delay_s{0.0};
    double fly_time_s{0.0};
    double total_hit_delay_s{0.0};
    double selected_x{0.0};
    double selected_y{0.0};
    double selected_z{0.0};
    double selected_yaw{0.0};
    double selected_view_angle{0.0};
    bool selected_front_facing{false};
    bool shootable{false};
    double command_yaw{0.0};
    double command_pitch{0.0};
    double target_yaw{0.0};
    double target_pitch{0.0};
    double planned_yaw{0.0};
    double planned_pitch{0.0};
    double planned_yaw_vel{0.0};
    double planned_pitch_vel{0.0};
    double planned_yaw_acc{0.0};
    double planned_pitch_acc{0.0};
    bool mpc_used{false};
    bool fire_allowed{false};
    double fire_thres_yaw{0.0};
    double fire_thres_pitch{0.0};
    double command_error_yaw{0.0};
    double command_error_pitch{0.0};
    double actual_gimbal_error_yaw{0.0};
    double actual_gimbal_error_pitch{0.0};
  };

  struct AimerShotEvent
  {
    uint64_t shot_id{0};
    uint64_t frame_id{0};
    uint64_t image_timestamp_us{0};
    uint64_t command_time_us{0};
    uint64_t expected_hit_time_us{0};
    uint8_t selected_armor_index{0};
    ArmorNumber target_id{ArmorNumber::INVALID};
    double command_yaw{0.0};
    double command_pitch{0.0};
    double actual_gimbal_yaw{0.0};
    double actual_gimbal_pitch{0.0};
    double bullet_speed{0.0};
    double fire_delay_s{0.0};
    double fly_time_est_s{0.0};
    FireReason fire_reason{FireReason::NO_TRACK};
  };

  Aimer(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app, Config cfg);

  void OnMonitor() override {}

 private:
  void BulletSpeedCallback(float bullet_speed_msg);
  void GimbalRotationCallback(LibXR::Quaternion<float> gimbal_rotation_msg);
  void TargetCallback(const ArmorTrackerTarget& target_msg);
  bool ShouldAutoFire(const Eigen::Vector3d& target_xyz, double selected_view_angle,
                      bool shootable, double yaw,
                      double pitch);
  void BuildTrajectoryMessage(const ArmorTrackerTarget& target_msg,
                              const Eigen::Vector3d& aim_point, double fly_time,
                              double launch_pitch, double bullet_speed, double yaw,
                              double pitch);
  void PublishDecisionAndMaybeShot();
  void WriteDecisionAudit();
  void WriteShotAudit(const AimerShotEvent& shot);
  void SetupGimbalPlanSolvers();
  bool BuildMpcGimbalPlan(const ArmorTrackerTarget& target_msg,
                          double bullet_speed, bool fire);
  void BuildFiniteDifferenceGimbalPlan(const ArmorTrackerTarget& target_msg,
                                       bool control, bool fire, double yaw,
                                       double pitch);
  void BuildGimbalPlan(const ArmorTrackerTarget& target_msg, bool control,
                       bool fire, double yaw, double pitch, double bullet_speed);
  void ResetGimbalPlanHistory();

 private:
  Config cfg_{};
  std::atomic<double> bullet_speed_{23.0};
  int lock_id_{-1};
  ArmorNumber last_target_id_{ArmorNumber::INVALID};
  uint64_t frame_index_{0};
  uint64_t shot_index_{0};
  bool has_last_command_{false};
  double last_command_yaw_{0.0};
  double last_command_pitch_{0.0};
  bool has_gimbal_rotation_{false};
  double last_fire_tolerance_rad_{0.0};
  double last_fire_command_error_rad_{0.0};
  double last_fire_command_pitch_error_rad_{0.0};
  double last_fire_gimbal_error_rad_{0.0};
  double last_fire_gimbal_pitch_error_rad_{0.0};
  double last_fire_gimbal_yaw_rad_{0.0};
  double last_fire_gimbal_pitch_rad_{0.0};
  LibXR::Quaternion<double> gimbal_rotation_{1.0, 0.0, 0.0, 0.0};
  bool planner_ready_{false};
  bool last_plan_mpc_{false};
  TinySolver* yaw_solver_{nullptr};
  TinySolver* pitch_solver_{nullptr};
  mutable LibXR::Mutex gimbal_rotation_lock_{};

  AimerMetrics metrics_msg_{};
  AimerTrajectory trajectory_msg_{};
  AimerDecision decision_msg_{};
  LibXR::EulerAngle<float> target_euler_msg_{};
  AimerSend send_msg_{};
  GimbalPlan gimbal_plan_msg_{};
  struct AuditFile
  {
    std::string path{};
    std::ofstream file{};
    bool open_failed{false};
  };
  AuditFile decision_audit_{};
  AuditFile shot_audit_{};

  LibXR::Topic::Domain aimer_domain_ = LibXR::Topic::Domain("aimer");
  LibXR::Topic metrics_topic_ =
      LibXR::Topic("metrics", sizeof(AimerMetrics), &aimer_domain_);
  LibXR::Topic trajectory_topic_ = LibXR::Topic(
      LibXR::Topic::FindOrCreate<AimerTrajectory>("trajectory", &aimer_domain_));
  LibXR::Topic decision_topic_ =
      LibXR::Topic("decision", sizeof(AimerDecision), &aimer_domain_);
  LibXR::Topic shot_event_topic_ =
      LibXR::Topic("shot_event", sizeof(AimerShotEvent), &aimer_domain_);

  LibXR::Topic::Domain tracker_domain_ = LibXR::Topic::Domain("tracker");
  LibXR::Topic target_euler_topic_ =
      LibXR::Topic("target_eulr", sizeof(LibXR::EulerAngle<float>), &tracker_domain_);
  LibXR::Topic fire_notify_topic_ =
      LibXR::Topic("fire_notify", sizeof(uint8_t), &tracker_domain_);
  LibXR::Topic send_topic_ =
      LibXR::Topic("send", sizeof(AimerSend), &tracker_domain_);
  LibXR::Topic gimbal_plan_topic_ =
      LibXR::Topic("gimbal_plan", sizeof(GimbalPlan), &tracker_domain_);
};
