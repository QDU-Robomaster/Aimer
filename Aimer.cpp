#include "Aimer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <limits>
#include <string_view>
#include <utility>
#include <vector>

#include "logger.hpp"

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double GRAVITY = 9.7833;
constexpr double MIN_HORIZONTAL_DISTANCE_M = 1e-4;
constexpr double PLAN_DEFAULT_DT_S = 0.01;
constexpr int PLAN_HALF_HORIZON = 50;
constexpr int PLAN_HORIZON = PLAN_HALF_HORIZON * 2;
constexpr int PLAN_SHOOT_OFFSET = 2;

using PlanTrajectory = Eigen::Matrix<double, 4, PLAN_HORIZON>;

double LimitRad(double angle)
{
  while (angle > PI)
  {
    angle -= 2.0 * PI;
  }
  while (angle < -PI)
  {
    angle += 2.0 * PI;
  }
  return angle;
}

uint64_t SecondsToMicros(double seconds)
{
  if (!std::isfinite(seconds) || seconds <= 0.0)
  {
    return 0;
  }
  return static_cast<uint64_t>(std::llround(seconds * 1.0e6));
}

template <typename Derived>
double HorizontalDistance(const Eigen::MatrixBase<Derived>& point)
{
  return std::hypot(point.x(), point.z());
}

template <typename Derived>
double BearingYaw(const Eigen::MatrixBase<Derived>& point)
{
  return std::atan2(point.z(), point.x());
}

template <typename Derived>
double BallisticHeight(const Eigen::MatrixBase<Derived>& point)
{
  return point.y();
}

const char* ToString(Aimer::Strategy strategy)
{
  switch (strategy)
  {
    case Aimer::Strategy::LOST:
      return "lost";
    case Aimer::Strategy::LOW_SPEED:
      return "low_speed";
    case Aimer::Strategy::MEDIUM_SPIN:
      return "medium_spin";
    case Aimer::Strategy::OUTPOST:
      return "outpost";
  }
  return "unknown";
}

const char* ToString(Aimer::SelectReason reason)
{
  switch (reason)
  {
    case Aimer::SelectReason::NONE:
      return "none";
    case Aimer::SelectReason::NEAREST_FRONT:
      return "nearest_front";
  }
  return "unknown";
}

const char* ToString(Aimer::SwitchReason reason)
{
  switch (reason)
  {
    case Aimer::SwitchReason::NONE:
      return "none";
    case Aimer::SwitchReason::NEW_TARGET:
      return "new_target";
    case Aimer::SwitchReason::NEAREST_CHANGED:
      return "nearest_changed";
  }
  return "unknown";
}

const char* ToString(Aimer::FireReason reason)
{
  switch (reason)
  {
    case Aimer::FireReason::DISABLED:
      return "disabled";
    case Aimer::FireReason::OK:
      return "ok";
    case Aimer::FireReason::NO_TRACK:
      return "no_track";
    case Aimer::FireReason::NO_GIMBAL:
      return "no_gimbal";
    case Aimer::FireReason::COMMAND_UNSTABLE:
      return "command_unstable";
    case Aimer::FireReason::GIMBAL_NOT_ALIGNED:
      return "gimbal_not_aligned";
    case Aimer::FireReason::NOT_SHOOTABLE:
      return "not_shootable";
    case Aimer::FireReason::BALLISTIC_UNSOLVABLE:
      return "ballistic_unsolvable";
  }
  return "unknown";
}

struct AimPoint
{
  bool valid{false};
  bool shootable{false};
  bool front_facing{false};
  uint8_t candidate_count{0};
  int armor_index{0};
  double view_angle{0.0};
  Aimer::Strategy strategy{Aimer::Strategy::LOST};
  Aimer::SelectReason selected_reason{Aimer::SelectReason::NONE};
  Aimer::SwitchReason switch_reason{Aimer::SwitchReason::NONE};
  Eigen::Vector4d xyza{Eigen::Vector4d::Zero()};
};

struct TrajectorySolution
{
  bool unsolvable{false};
  double fly_time{0.0};
  double pitch{0.0};
};

struct AimCommand
{
  bool valid{false};
  Eigen::Vector2d yaw_pitch{Eigen::Vector2d::Zero()};
  double fly_time{0.0};
  AimPoint aim_point{};
};

struct PredictedTarget
{
  ArmorTrackerTarget msg{};

  void Predict(double dt)
  {
    msg.position.x() += msg.velocity.x() * dt;
    msg.position.y() += msg.velocity.y() * dt;
    msg.position.z() += msg.velocity.z() * dt;
    msg.yaw = LimitRad(msg.yaw + msg.v_yaw * dt);
  }

  std::vector<Eigen::Vector4d> GetArmorXYZAList() const
  {
    std::vector<Eigen::Vector4d> armor_xyza_list;
    armor_xyza_list.reserve(static_cast<std::size_t>(std::max(msg.armors_num, 0)));

    for (int index = 0; index < msg.armors_num; ++index)
    {
      const double angle = LimitRad(msg.yaw + index * 2.0 * PI / msg.armors_num);
      const bool use_length_height = (msg.armors_num == 4) && (index == 1 || index == 3);
      const double radius = use_length_height ? msg.radius_2 : msg.radius_1;
      const double armor_x = msg.position.x() + radius * std::cos(angle);
      const double armor_y =
          use_length_height ? msg.position.y() + msg.dz : msg.position.y();
      const double armor_z = msg.position.z() + radius * std::sin(angle);
      armor_xyza_list.push_back({armor_x, armor_y, armor_z, angle});
    }

    return armor_xyza_list;
  }
};

Aimer::Strategy SelectStrategy(const Aimer::Config& cfg,
                               const ArmorTrackerTarget& target)
{
  if (!target.tracking)
  {
    return Aimer::Strategy::LOST;
  }
  if (target.id == ArmorNumber::OUTPOST)
  {
    return Aimer::Strategy::OUTPOST;
  }
  const double abs_v_yaw = std::abs(target.v_yaw);
  if (abs_v_yaw <= cfg.yaw_rate_threshold)
  {
    return Aimer::Strategy::LOW_SPEED;
  }
  return Aimer::Strategy::MEDIUM_SPIN;
}

double FixedPredictDelay(const Aimer::Config& cfg, const ArmorTrackerTarget& target)
{
  const bool high_speed = std::abs(target.v_yaw) > cfg.yaw_rate_threshold;
  return cfg.image_to_now_s + cfg.vision_to_command_delay_s +
         cfg.command_transport_delay_s + cfg.gimbal_response_delay_s +
         (high_speed ? cfg.high_speed_extra_predict_s
                     : cfg.low_speed_extra_predict_s);
}

double ViewAngle(const ArmorTrackerTarget& target, const Eigen::Vector4d& xyza)
{
  const double center_yaw = BearingYaw(target.position);
  return LimitRad(xyza[3] - center_yaw);
}

bool IsFrontFacing(const Aimer::Config& cfg, const ArmorTrackerTarget& target,
                   const Eigen::Vector4d& xyza)
{
  (void)cfg;
  return std::abs(ViewAngle(target, xyza)) <= 75.0 * DEG2RAD;
}

AimPoint BuildAimPoint(const Aimer::Config& cfg, const PredictedTarget& target,
                       int armor_index, const Eigen::Vector4d& xyza,
                       Aimer::Strategy strategy, Aimer::SelectReason reason,
                       Aimer::SwitchReason switch_reason, bool shootable,
                       uint8_t candidate_count)
{
  AimPoint out;
  out.valid = true;
  out.shootable = shootable;
  out.front_facing = IsFrontFacing(cfg, target.msg, xyza);
  out.candidate_count = candidate_count;
  out.armor_index = armor_index;
  out.view_angle = ViewAngle(target.msg, xyza);
  out.strategy = strategy;
  out.selected_reason = reason;
  out.switch_reason = switch_reason;
  out.xyza = xyza;
  return out;
}

std::pair<double, double> DynamicFireThreshold(const Aimer::Config& cfg,
                                               const Eigen::Vector3d& target_xyz,
                                               double selected_view_angle)
{
  const double horizontal_distance =
      std::max(MIN_HORIZONTAL_DISTANCE_M, HorizontalDistance(target_xyz));
  const double distance = std::max(MIN_HORIZONTAL_DISTANCE_M, target_xyz.norm());
  const double facing_scale =
      std::clamp(std::cos(std::abs(selected_view_angle)), 0.25, 1.0);
  const double yaw_half =
      std::atan2(0.5 * cfg.armor_width_m * facing_scale, horizontal_distance);
  const double pitch_half = std::atan2(0.5 * cfg.armor_height_m, distance);
  const double spread_yaw = std::atan2(cfg.bullet_spread_m, horizontal_distance);
  const double spread_pitch = std::atan2(cfg.bullet_spread_m, distance);
  const double yaw_threshold =
      std::clamp(yaw_half - spread_yaw, cfg.min_fire_threshold,
                 cfg.max_fire_threshold);
  const double pitch_threshold =
      std::clamp(pitch_half - spread_pitch, cfg.min_fire_threshold,
                 cfg.max_fire_threshold);
  return {yaw_threshold, pitch_threshold};
}

TrajectorySolution SolveTrajectoryPitch(double bullet_speed, double horizontal_distance,
                                        double target_height)
{
  TrajectorySolution solution;

  if (bullet_speed <= 0.0 || horizontal_distance <= MIN_HORIZONTAL_DISTANCE_M)
  {
    solution.unsolvable = true;
    return solution;
  }

  const double a = GRAVITY * horizontal_distance * horizontal_distance /
                   (2.0 * bullet_speed * bullet_speed);
  const double b = -horizontal_distance;
  const double c = a + target_height;
  const double delta = b * b - 4.0 * a * c;

  if (delta < 0.0)
  {
    solution.unsolvable = true;
    return solution;
  }

  const double tan_pitch_1 = (-b + std::sqrt(delta)) / (2.0 * a);
  const double tan_pitch_2 = (-b - std::sqrt(delta)) / (2.0 * a);
  const double pitch_1 = std::atan(tan_pitch_1);
  const double pitch_2 = std::atan(tan_pitch_2);
  const double fly_time_1 = horizontal_distance / (bullet_speed * std::cos(pitch_1));
  const double fly_time_2 = horizontal_distance / (bullet_speed * std::cos(pitch_2));

  solution.unsolvable = false;
  solution.pitch = (fly_time_1 < fly_time_2) ? pitch_1 : pitch_2;
  solution.fly_time = (fly_time_1 < fly_time_2) ? fly_time_1 : fly_time_2;
  return solution;
}

[[maybe_unused]] std::string_view ArmorNumberToString(ArmorNumber number)
{
  const std::size_t index = static_cast<std::size_t>(number);
  if (index >= ARMOR_NUMBER_NAMES.size())
  {
    return std::string_view{"invalid"};
  }
  return ARMOR_NUMBER_NAMES[index];
}

AimPoint ChooseNearestArmor(const Aimer::Config& cfg, const PredictedTarget& target,
                            const std::vector<Eigen::Vector4d>& armor_xyza_list,
                            int& lock_id, Aimer::Strategy strategy)
{
  int nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (int index = 0; index < static_cast<int>(armor_xyza_list.size()); ++index)
  {
    const double distance = HorizontalDistance(armor_xyza_list[index].head<3>());
    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  const int old_lock_id = lock_id;
  lock_id = nearest_index;
  Aimer::SwitchReason switch_reason = Aimer::SwitchReason::NONE;
  if (old_lock_id >= 0 && old_lock_id != nearest_index)
  {
    switch_reason = Aimer::SwitchReason::NEAREST_CHANGED;
  }
  return BuildAimPoint(
      cfg, target, nearest_index, armor_xyza_list[nearest_index], strategy,
      Aimer::SelectReason::NEAREST_FRONT, switch_reason, true,
      static_cast<uint8_t>(std::min<std::size_t>(armor_xyza_list.size(), 255U)));
}

AimPoint ChooseAimPoint(const Aimer::Config& cfg, const PredictedTarget& target,
                        int& lock_id)
{
  const Aimer::Strategy strategy = SelectStrategy(cfg, target.msg);
  if (strategy == Aimer::Strategy::LOST)
  {
    lock_id = -1;
    return {};
  }

  const auto armor_xyza_list = target.GetArmorXYZAList();
  if (armor_xyza_list.empty())
  {
    lock_id = -1;
    return {};
  }

  return ChooseNearestArmor(cfg, target, armor_xyza_list, lock_id, strategy);
}

AimCommand ComputeNearestAimCommand(const Aimer::Config& cfg,
                                    const PredictedTarget& target,
                                    double bullet_speed)
{
  AimCommand command;
  const auto armor_xyza_list = target.GetArmorXYZAList();
  if (armor_xyza_list.empty())
  {
    return command;
  }

  int nearest_lock_id = -1;
  command.aim_point = ChooseNearestArmor(cfg, target, armor_xyza_list,
                                         nearest_lock_id,
                                         Aimer::Strategy::LOW_SPEED);

  const Eigen::Vector3d xyz = command.aim_point.xyza.head<3>();
  const double horizontal_distance = HorizontalDistance(xyz);
  const auto trajectory =
      SolveTrajectoryPitch(bullet_speed, horizontal_distance, BallisticHeight(xyz));
  if (trajectory.unsolvable)
  {
    return command;
  }

  command.valid = true;
  command.fly_time = trajectory.fly_time;
  command.yaw_pitch.x() =
      LimitRad(BearingYaw(xyz) + cfg.yaw_offset * DEG2RAD);
  command.yaw_pitch.y() = -(trajectory.pitch + cfg.pitch_offset * DEG2RAD);
  return command;
}

bool BuildReferenceTrajectory(const Aimer::Config& cfg,
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
}  // namespace

Aimer::Aimer(LibXR::HardwareContainer&, LibXR::ApplicationManager& app, Config cfg)
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

void Aimer::BulletSpeedCallback(float bullet_speed_msg)
{
  if (!std::isnan(bullet_speed_msg))
  {
    bullet_speed_.store(bullet_speed_msg, std::memory_order_relaxed);
  }
}

void Aimer::GimbalRotationCallback(LibXR::Quaternion<float> gimbal_rotation_msg)
{
  LibXR::Mutex::LockGuard lock(gimbal_rotation_lock_);
  gimbal_rotation_ =
      LibXR::Quaternion<double>(gimbal_rotation_msg.w(), gimbal_rotation_msg.x(),
                                gimbal_rotation_msg.y(), gimbal_rotation_msg.z());
  has_gimbal_rotation_ = true;
}

bool Aimer::ShouldAutoFire(const Eigen::Vector3d& target_xyz,
                           double selected_view_angle, bool shootable, double yaw,
                           double pitch)
{
  const auto [yaw_threshold, pitch_threshold] =
      DynamicFireThreshold(cfg_, target_xyz, selected_view_angle);
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
    last_fire_gimbal_error_rad_ = std::abs(LimitRad(gimbal_yaw - yaw));
    last_fire_gimbal_pitch_error_rad_ = std::abs(LimitRad(gimbal_pitch - pitch));
    metrics_msg_.fire_reason = FireReason::COMMAND_UNSTABLE;
    last_command_yaw_ = yaw;
    last_command_pitch_ = pitch;
    has_last_command_ = true;
    return false;
  }

  last_fire_command_error_rad_ = std::abs(LimitRad(last_command_yaw_ - yaw));
  last_fire_command_pitch_error_rad_ =
      std::abs(LimitRad(last_command_pitch_ - pitch));
  last_fire_gimbal_error_rad_ = std::abs(LimitRad(gimbal_yaw - yaw));
  last_fire_gimbal_pitch_error_rad_ = std::abs(LimitRad(gimbal_pitch - pitch));
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

void Aimer::BuildTrajectoryMessage(const ArmorTrackerTarget& target_msg,
                                   const Eigen::Vector3d& aim_point, double fly_time,
                                   double launch_pitch, double bullet_speed, double yaw,
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

  const double horizontal_distance = HorizontalDistance(aim_point);
  if (fly_time <= 0.0 || bullet_speed <= 0.0 ||
      horizontal_distance <= MIN_HORIZONTAL_DISTANCE_M)
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
    const double y = v_vertical * t - 0.5 * GRAVITY * t * t;
    trajectory_msg_.points[index] =
        LibXR::Position<double>(dir_x * s, y, dir_z * s);
  }
}

void Aimer::WriteDecisionAudit()
{
  if (decision_audit_.path.empty())
  {
    return;
  }
  if (!decision_audit_.file.is_open())
  {
    decision_audit_.file.open(decision_audit_.path, std::ios::out | std::ios::trunc);
    if (!decision_audit_.file)
    {
      if (!decision_audit_.open_failed)
      {
        XR_LOG_ERROR("Aimer failed to open decision audit: %s",
                     decision_audit_.path.c_str());
        decision_audit_.open_failed = true;
      }
      return;
    }
    decision_audit_.file << std::setprecision(9);
    decision_audit_.file
        << "frame_id\timage_timestamp_us\taimer_receive_time_us\tpredict_time_us\t"
        << "expected_hit_time_us\ttarget_tracking\tvalid\tconverged\t"
        << "candidate_count\tselected_armor_index\ttarget_id\t"
        << "strategy\tselected_reason\tswitch_reason\tfire_reason\t"
        << "fixed_delay_s\tfire_delay_s\tfly_time_s\ttotal_hit_delay_s\t"
        << "selected_x\tselected_y\tselected_z\tselected_yaw\tselected_view_angle\t"
        << "selected_front_facing\tshootable\tcommand_yaw\tcommand_pitch\t"
        << "target_yaw\ttarget_pitch\tplanned_yaw\tplanned_pitch\t"
        << "planned_yaw_vel\tplanned_pitch_vel\tplanned_yaw_acc\tplanned_pitch_acc\t"
        << "mpc_used\tfire_allowed\tfire_thres_yaw\tfire_thres_pitch\t"
        << "command_error_yaw\tcommand_error_pitch\tactual_gimbal_error_yaw\t"
        << "actual_gimbal_error_pitch\n";
  }

  decision_audit_.file
      << decision_msg_.frame_id << '\t' << decision_msg_.image_timestamp_us << '\t'
      << decision_msg_.aimer_receive_time_us << '\t' << decision_msg_.predict_time_us
      << '\t' << decision_msg_.expected_hit_time_us << '\t'
      << (decision_msg_.target_tracking ? 1 : 0) << '\t'
      << (decision_msg_.valid ? 1 : 0) << '\t'
      << (decision_msg_.converged ? 1 : 0) << '\t'
      << static_cast<int>(decision_msg_.candidate_count) << '\t'
      << static_cast<int>(decision_msg_.selected_armor_index) << '\t'
      << static_cast<int>(decision_msg_.target_id) << '\t'
      << ToString(decision_msg_.strategy) << '\t'
      << ToString(decision_msg_.selected_reason) << '\t'
      << ToString(decision_msg_.switch_reason) << '\t'
      << ToString(decision_msg_.fire_reason) << '\t'
      << decision_msg_.fixed_delay_s << '\t' << decision_msg_.fire_delay_s << '\t'
      << decision_msg_.fly_time_s << '\t' << decision_msg_.total_hit_delay_s
      << '\t' << decision_msg_.selected_x << '\t' << decision_msg_.selected_y
      << '\t' << decision_msg_.selected_z << '\t' << decision_msg_.selected_yaw
      << '\t' << decision_msg_.selected_view_angle << '\t'
      << (decision_msg_.selected_front_facing ? 1 : 0) << '\t'
      << (decision_msg_.shootable ? 1 : 0) << '\t'
      << decision_msg_.command_yaw << '\t' << decision_msg_.command_pitch << '\t'
      << decision_msg_.target_yaw << '\t' << decision_msg_.target_pitch << '\t'
      << decision_msg_.planned_yaw << '\t' << decision_msg_.planned_pitch << '\t'
      << decision_msg_.planned_yaw_vel << '\t' << decision_msg_.planned_pitch_vel
      << '\t' << decision_msg_.planned_yaw_acc << '\t'
      << decision_msg_.planned_pitch_acc << '\t'
      << (decision_msg_.mpc_used ? 1 : 0) << '\t'
      << (decision_msg_.fire_allowed ? 1 : 0) << '\t'
      << decision_msg_.fire_thres_yaw << '\t'
      << decision_msg_.fire_thres_pitch << '\t'
      << decision_msg_.command_error_yaw << '\t'
      << decision_msg_.command_error_pitch << '\t'
      << decision_msg_.actual_gimbal_error_yaw << '\t'
      << decision_msg_.actual_gimbal_error_pitch << '\n';
  decision_audit_.file.flush();
}

void Aimer::WriteShotAudit(const Aimer::AimerShotEvent& shot)
{
  if (shot_audit_.path.empty())
  {
    return;
  }
  if (!shot_audit_.file.is_open())
  {
    shot_audit_.file.open(shot_audit_.path, std::ios::out | std::ios::trunc);
    if (!shot_audit_.file)
    {
      if (!shot_audit_.open_failed)
      {
        XR_LOG_ERROR("Aimer failed to open shot audit: %s",
                     shot_audit_.path.c_str());
        shot_audit_.open_failed = true;
      }
      return;
    }
    shot_audit_.file << std::setprecision(9);
    shot_audit_.file
        << "shot_id\tframe_id\timage_timestamp_us\tcommand_time_us\t"
        << "expected_hit_time_us\tselected_armor_index\ttarget_id\tcommand_yaw\t"
        << "command_pitch\tactual_gimbal_yaw\tactual_gimbal_pitch\tbullet_speed\t"
        << "fire_delay_s\tfly_time_est_s\tfire_reason\n";
  }
  shot_audit_.file << shot.shot_id << '\t' << shot.frame_id << '\t'
                   << shot.image_timestamp_us << '\t' << shot.command_time_us
                   << '\t' << shot.expected_hit_time_us << '\t'
                   << static_cast<int>(shot.selected_armor_index) << '\t'
                   << static_cast<int>(shot.target_id) << '\t'
                   << shot.command_yaw << '\t' << shot.command_pitch << '\t'
                   << shot.actual_gimbal_yaw << '\t' << shot.actual_gimbal_pitch
                   << '\t' << shot.bullet_speed << '\t' << shot.fire_delay_s
                   << '\t' << shot.fly_time_est_s << '\t'
                   << ToString(shot.fire_reason) << '\n';
  shot_audit_.file.flush();
}

void Aimer::PublishDecisionAndMaybeShot()
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

void Aimer::SetupGimbalPlanSolvers()
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
    a << 1.0, PLAN_DEFAULT_DT_S, 0.0, 1.0;
    Eigen::MatrixXd b(2, 1);
    b << 0.0, PLAN_DEFAULT_DT_S;
    Eigen::VectorXd f(2);
    f << 0.0, 0.0;
    Eigen::MatrixXd q(2, 2);
    q << q_pos, 0.0, 0.0, q_vel;
    Eigen::MatrixXd r(1, 1);
    r << r_acc;

    if (tiny_setup(solver, a, b, f, q, r, 1.0, 2, 1, PLAN_HORIZON, 0) != 0)
    {
      return false;
    }

    Eigen::MatrixXd x_min =
        Eigen::MatrixXd::Constant(2, PLAN_HORIZON, -1.0e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, PLAN_HORIZON, 1.0e17);
    Eigen::MatrixXd u_min =
        Eigen::MatrixXd::Constant(1, PLAN_HORIZON - 1, -max_acc);
    Eigen::MatrixXd u_max =
        Eigen::MatrixXd::Constant(1, PLAN_HORIZON - 1, max_acc);
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
        PLAN_HORIZON, PLAN_DEFAULT_DT_S, cfg_.max_yaw_acc, cfg_.max_pitch_acc,
        cfg_.mpc_max_iter);
  }
  else
  {
    XR_LOG_WARN("Aimer TinyMPC gimbal_plan setup failed; finite-difference fallback active");
  }
}

void Aimer::ResetGimbalPlanHistory()
{
  last_plan_mpc_ = false;
}

bool Aimer::BuildMpcGimbalPlan(const ArmorTrackerTarget& target_msg,
                               double bullet_speed, bool fire)
{
  if (!planner_ready_ || yaw_solver_ == nullptr || pitch_solver_ == nullptr)
  {
    return false;
  }

  PlanTrajectory reference{};
  double yaw0 = 0.0;
  if (!BuildReferenceTrajectory(cfg_, target_msg, metrics_msg_.delay_time_s,
                                bullet_speed, reference, yaw0))
  {
    return false;
  }

  Eigen::VectorXd x0(2);
  x0 << reference(0, 0), reference(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = reference.block(0, 0, 2, PLAN_HORIZON);
  tiny_solve(yaw_solver_);

  x0 << reference(2, 0), reference(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = reference.block(2, 0, 2, PLAN_HORIZON);
  tiny_solve(pitch_solver_);

  const int output_index = PLAN_HALF_HORIZON;
  const double target_yaw = LimitRad(reference(0, output_index) + yaw0);
  const double target_pitch = reference(2, output_index);
  const double planned_yaw =
      LimitRad(yaw_solver_->work->x(0, output_index) + yaw0);
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
      std::min(PLAN_HORIZON - 1, PLAN_HALF_HORIZON + PLAN_SHOOT_OFFSET);
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

void Aimer::BuildFiniteDifferenceGimbalPlan(const ArmorTrackerTarget& target_msg,
                                            bool control, bool fire, double yaw,
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

void Aimer::BuildGimbalPlan(const ArmorTrackerTarget& target_msg, bool control,
                            bool fire, double yaw, double pitch, double bullet_speed)
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

void Aimer::TargetCallback(const ArmorTrackerTarget& target_msg)
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
  metrics_msg_.strategy = SelectStrategy(cfg_, target_msg);
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

  const double delay_time = target_msg.tracking ? FixedPredictDelay(cfg_, target_msg) : 0.0;
  metrics_msg_.delay_time_s = delay_time;
  decision_msg_.fixed_delay_s = delay_time;
  decision_msg_.fire_delay_s = cfg_.fire_delay_s;
  decision_msg_.predict_time_us =
      target_msg.image_timestamp_us + SecondsToMicros(delay_time);

  if (!target_msg.tracking)
  {
    has_last_command_ = false;
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  PredictedTarget base_target{target_msg};
  base_target.Predict(delay_time);

  auto record_aim_point = [this](const AimPoint& aim_point)
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

  AimPoint debug_aim_point = ChooseAimPoint(cfg_, base_target, lock_id_);

  if (!debug_aim_point.valid)
  {
    metrics_msg_.fire_reason = FireReason::NOT_SHOOTABLE;
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }
  record_aim_point(debug_aim_point);

  const Eigen::Vector3d first_xyz = debug_aim_point.xyza.head<3>();
  const double first_horizontal_distance = HorizontalDistance(first_xyz);
  const auto first_trajectory = SolveTrajectoryPitch(
      bullet_speed, first_horizontal_distance, BallisticHeight(first_xyz));

  if (first_trajectory.unsolvable)
  {
    metrics_msg_.fire_reason = FireReason::BALLISTIC_UNSOLVABLE;
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  PredictedTarget hit_target = base_target;
  hit_target.Predict(first_trajectory.fly_time);
  debug_aim_point = ChooseAimPoint(cfg_, hit_target, lock_id_);
  if (!debug_aim_point.valid)
  {
    metrics_msg_.fire_reason = FireReason::NOT_SHOOTABLE;
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  record_aim_point(debug_aim_point);

  const Eigen::Vector3d hit_xyz = debug_aim_point.xyza.head<3>();
  const double hit_horizontal_distance = HorizontalDistance(hit_xyz);
  const auto trajectory = SolveTrajectoryPitch(
      bullet_speed, hit_horizontal_distance, BallisticHeight(hit_xyz));
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
      target_msg.image_timestamp_us + SecondsToMicros(metrics_msg_.total_hit_delay_s);

  const Eigen::Vector3d final_xyz = debug_aim_point.xyza.head<3>();
  const double yaw = LimitRad(BearingYaw(final_xyz) + cfg_.yaw_offset * DEG2RAD);
  const double pitch = -(trajectory.pitch + cfg_.pitch_offset * DEG2RAD);

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
        ArmorNumberToString(target_msg.id).data(), metrics_msg_.valid ? 1 : 0,
        metrics_msg_.converged ? 1 : 0, metrics_msg_.is_fire ? 1 : 0,
        metrics_msg_.iteration_count, ToString(metrics_msg_.strategy),
        ToString(metrics_msg_.selected_reason), ToString(metrics_msg_.switch_reason),
        ToString(metrics_msg_.fire_reason), metrics_msg_.delay_time_s, metrics_msg_.fly_time_s,
        metrics_msg_.yaw, last_fire_gimbal_yaw_rad_,
        last_fire_command_error_rad_ / DEG2RAD, last_fire_gimbal_error_rad_ / DEG2RAD,
        last_fire_tolerance_rad_ / DEG2RAD, last_plan_mpc_ ? 1 : 0,
        static_cast<double>(gimbal_plan_msg_.yaw),
        static_cast<double>(gimbal_plan_msg_.pitch),
        static_cast<double>(gimbal_plan_msg_.yaw_vel),
        static_cast<double>(gimbal_plan_msg_.pitch_vel),
        static_cast<double>(gimbal_plan_msg_.yaw_acc),
        static_cast<double>(gimbal_plan_msg_.pitch_acc), metrics_msg_.latency_ms);
  }
}
