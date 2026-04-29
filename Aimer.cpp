#include "Aimer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
constexpr double OUTPOST_COMING_ANGLE_DEG = 70.0;
constexpr double OUTPOST_LEAVING_ANGLE_DEG = 30.0;
constexpr int MAX_ITERATION_COUNT = 10;
constexpr double FLY_TIME_CONVERGENCE_S = 0.001;
constexpr double MIN_HORIZONTAL_DISTANCE_M = 1e-4;

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

struct AimPoint
{
  bool valid{false};
  int armor_index{0};
  Eigen::Vector4d xyza{Eigen::Vector4d::Zero()};
};

struct TrajectorySolution
{
  bool unsolvable{false};
  double fly_time{0.0};
  double pitch{0.0};
};

struct PredictedTarget
{
  SolveTrajectory::Target msg{};

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
      const double angle =
          LimitRad(msg.yaw + index * 2.0 * PI / msg.armors_num);
      const bool use_length_height = (msg.armors_num == 4) && (index == 1 || index == 3);
      const double radius = use_length_height ? msg.radius_2 : msg.radius_1;
      const double armor_x = msg.position.x() - radius * std::cos(angle);
      const double armor_y = msg.position.y() - radius * std::sin(angle);
      const double armor_z = use_length_height ? msg.position.z() + msg.dz
                                               : msg.position.z();
      armor_xyza_list.push_back({armor_x, armor_y, armor_z, angle});
    }

    return armor_xyza_list;
  }
};

TrajectorySolution SolveTrajectoryPitch(double bullet_speed, double horizontal_distance,
                                        double target_height)
{
  TrajectorySolution solution;

  if (bullet_speed <= 0.0 || horizontal_distance <= MIN_HORIZONTAL_DISTANCE_M)
  {
    solution.unsolvable = true;
    return solution;
  }

  const double a =
      GRAVITY * horizontal_distance * horizontal_distance /
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

std::string_view ArmorNumberToString(ArmorNumber number)
{
  const std::size_t index = static_cast<std::size_t>(number);
  if (index >= ARMOR_NUMBER_NAMES.size())
  {
    return std::string_view{"invalid"};
  }
  return ARMOR_NUMBER_NAMES[index];
}

AimPoint ChooseNearestArmor(const std::vector<Eigen::Vector4d>& armor_xyza_list,
                            int& lock_id)
{
  int nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (int index = 0; index < static_cast<int>(armor_xyza_list.size()); ++index)
  {
    const double distance =
        std::hypot(armor_xyza_list[index][0], armor_xyza_list[index][1]);
    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_index = index;
    }
  }

  lock_id = nearest_index;
  return {true, nearest_index, armor_xyza_list[nearest_index]};
}

AimPoint ChooseRotatingArmor(const Aimer::Config& cfg, const PredictedTarget& target,
                             const std::vector<Eigen::Vector4d>& armor_xyza_list,
                             int& lock_id)
{
  const bool is_outpost = target.msg.id == ArmorNumber::OUTPOST;
  const double center_yaw =
      std::atan2(target.msg.position.y(), target.msg.position.x());
  const double coming_angle =
      (is_outpost ? OUTPOST_COMING_ANGLE_DEG : cfg.comming_angle) * DEG2RAD;
  const double leaving_angle =
      (is_outpost ? OUTPOST_LEAVING_ANGLE_DEG : cfg.leaving_angle) * DEG2RAD;

  for (int index = 0; index < static_cast<int>(armor_xyza_list.size()); ++index)
  {
    const double delta_angle = LimitRad(armor_xyza_list[index][3] - center_yaw);
    if (std::abs(delta_angle) > coming_angle)
    {
      continue;
    }

    if ((target.msg.v_yaw > 0.0 && delta_angle < leaving_angle) ||
        (target.msg.v_yaw < 0.0 && delta_angle > -leaving_angle))
    {
      lock_id = index;
      return {true, index, armor_xyza_list[index]};
    }
  }

  lock_id = -1;
  return {};
}

AimPoint ChooseAimPoint(const Aimer::Config& cfg, const PredictedTarget& target,
                        int& lock_id)
{
  const auto armor_xyza_list = target.GetArmorXYZAList();
  if (armor_xyza_list.empty())
  {
    lock_id = -1;
    return {};
  }

  // 低速普通目标不抢切面，直接瞄最近装甲板；高速目标和前哨站按旋转窗口选面。
  const bool is_outpost = target.msg.id == ArmorNumber::OUTPOST;
  if (!is_outpost && std::abs(target.msg.v_yaw) <= cfg.yaw_rate_threshold)
  {
    return ChooseNearestArmor(armor_xyza_list, lock_id);
  }

  return ChooseRotatingArmor(cfg, target, armor_xyza_list, lock_id);
}
}  // namespace

Aimer::Aimer(LibXR::HardwareContainer&, LibXR::ApplicationManager& app, Config cfg)
    : cfg_(std::move(cfg)), bullet_speed_(cfg_.default_bullet_speed)
{
  LibXR::Topic::Domain tracker_domain("tracker");
  LibXR::Topic target_topic =
      LibXR::Topic::FindOrCreate<SolveTrajectory::Target>("target", &tracker_domain);
  auto target_callback = LibXR::Topic::Callback::Create(
      [](bool, Aimer* self, LibXR::RawData& data)
      {
        auto* target_msg = reinterpret_cast<SolveTrajectory::Target*>(data.addr_);
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
        auto* gimbal_rotation_msg = reinterpret_cast<LibXR::Quaternion<float>*>(data.addr_);
        self->GimbalRotationCallback(*gimbal_rotation_msg);
      },
      this);
  gimbal_rotation_topic.RegisterCallback(gimbal_rotation_callback);

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
  gimbal_rotation_ = LibXR::Quaternion<double>(gimbal_rotation_msg.w(), gimbal_rotation_msg.x(),
                                               gimbal_rotation_msg.y(),
                                               gimbal_rotation_msg.z());
  has_gimbal_rotation_ = true;
}

bool Aimer::ShouldAutoFire(const Eigen::Vector3d& target_xyz, double yaw)
{
  if (!cfg_.auto_fire || !has_last_command_)
  {
    last_fire_tolerance_rad_ = 0.0;
    last_fire_command_error_rad_ = 0.0;
    last_fire_gimbal_error_rad_ = 0.0;
    last_fire_gimbal_yaw_rad_ = 0.0;
    last_command_yaw_ = yaw;
    has_last_command_ = true;
    return false;
  }

  const double target_distance = std::hypot(target_xyz.x(), target_xyz.y());
  const double tolerance =
      (target_distance > cfg_.judge_distance ? cfg_.second_tolerance : cfg_.first_tolerance) *
      DEG2RAD;
  last_fire_tolerance_rad_ = tolerance;

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
    last_fire_gimbal_error_rad_ = 0.0;
    last_fire_gimbal_yaw_rad_ = 0.0;
    last_command_yaw_ = yaw;
    has_last_command_ = true;
    return false;
  }

  const auto GIMBAL_EULER = gimbal_rotation.ToEulerAngleZYX();
  gimbal_yaw = GIMBAL_EULER[2];
  last_fire_gimbal_yaw_rad_ = gimbal_yaw;

  last_fire_command_error_rad_ = std::abs(LimitRad(last_command_yaw_ - yaw));
  last_fire_gimbal_error_rad_ = std::abs(LimitRad(gimbal_yaw - last_command_yaw_));

  const bool command_stable = last_fire_command_error_rad_ < tolerance * 2.0;
  const bool gimbal_aligned = last_fire_gimbal_error_rad_ < tolerance;

  last_command_yaw_ = yaw;
  has_last_command_ = true;
  return command_stable && gimbal_aligned;
}

void Aimer::TargetCallback(const SolveTrajectory::Target& target_msg)
{
  const auto start_time = std::chrono::steady_clock::now();
  auto publish_outputs = [&]()
  {
    metrics_msg_.latency_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  start_time)
            .count();
    metrics_topic_.Publish(metrics_msg_);
    target_euler_topic_.Publish(target_euler_msg_);
    send_topic_.Publish(send_msg_);
  };

  ++frame_index_;
  metrics_msg_ = {};
  metrics_msg_.frame_index = frame_index_;
  metrics_msg_.target_tracking = target_msg.tracking;
  metrics_msg_.target_id = target_msg.id;

  if (target_msg.id != last_target_id_)
  {
    lock_id_ = -1;
    has_last_command_ = false;
    last_target_id_ = target_msg.id;
  }

  send_msg_ = {};
  target_euler_msg_ = LibXR::EulerAngle<float>();

  double bullet_speed = bullet_speed_.load(std::memory_order_relaxed);
  if (std::isnan(bullet_speed) || bullet_speed < cfg_.min_valid_bullet_speed)
  {
    bullet_speed = cfg_.default_bullet_speed;
  }
  metrics_msg_.bullet_speed = bullet_speed;

  if (!target_msg.tracking)
  {
    has_last_command_ = false;
    publish_outputs();
    return;
  }

  const double delay_time =
      (std::abs(target_msg.v_yaw) > cfg_.yaw_rate_threshold)
          ? cfg_.high_speed_delay_time
          : cfg_.low_speed_delay_time;
  metrics_msg_.delay_time_s = delay_time;

  PredictedTarget base_target{target_msg};
  base_target.Predict(delay_time);

  AimPoint debug_aim_point = ChooseAimPoint(cfg_, base_target, lock_id_);

  if (!debug_aim_point.valid)
  {
    publish_outputs();
    return;
  }

  const Eigen::Vector3d xyz_0 = debug_aim_point.xyza.head<3>();
  const double horizontal_distance_0 =
      std::hypot(xyz_0.x(), xyz_0.y());
  auto trajectory = SolveTrajectoryPitch(bullet_speed, horizontal_distance_0, xyz_0.z());

  if (trajectory.unsolvable)
  {
    publish_outputs();
    return;
  }

  bool converged = false;
  double prev_fly_time = trajectory.fly_time;
  metrics_msg_.selected_armor_index =
      static_cast<uint32_t>(std::max(debug_aim_point.armor_index, 0));

  for (int iteration = 0; iteration < MAX_ITERATION_COUNT; ++iteration)
  {
    PredictedTarget iter_target = base_target;
    iter_target.Predict(prev_fly_time);

    AimPoint aim_point = ChooseAimPoint(cfg_, iter_target, lock_id_);
    if (!aim_point.valid)
    {
      debug_aim_point = {};
      break;
    }

    debug_aim_point = aim_point;
    metrics_msg_.selected_armor_index =
        static_cast<uint32_t>(std::max(debug_aim_point.armor_index, 0));

    const Eigen::Vector3d xyz = debug_aim_point.xyza.head<3>();
    const double horizontal_distance = std::hypot(xyz.x(), xyz.y());
    trajectory = SolveTrajectoryPitch(bullet_speed, horizontal_distance, xyz.z());
    if (trajectory.unsolvable)
    {
      debug_aim_point = {};
      break;
    }

    metrics_msg_.iteration_count = static_cast<uint32_t>(iteration + 1);
    if (std::abs(trajectory.fly_time - prev_fly_time) < FLY_TIME_CONVERGENCE_S)
    {
      converged = true;
      break;
    }
    prev_fly_time = trajectory.fly_time;
  }

  if (!debug_aim_point.valid || trajectory.unsolvable)
  {
    publish_outputs();
    return;
  }

  metrics_msg_.valid = true;
  metrics_msg_.converged = converged;
  metrics_msg_.fly_time_s = trajectory.fly_time;

  const Eigen::Vector3d final_xyz = debug_aim_point.xyza.head<3>();
  const double yaw = std::atan2(final_xyz.y(), final_xyz.x()) +
                     cfg_.yaw_offset * DEG2RAD;
  const double pitch = -(trajectory.pitch + cfg_.pitch_offset * DEG2RAD);

  metrics_msg_.yaw = yaw;
  metrics_msg_.pitch = pitch;

  target_euler_msg_.Pitch() = static_cast<float>(pitch);
  target_euler_msg_.Yaw() = static_cast<float>(yaw);

  send_msg_.is_fire = ShouldAutoFire(final_xyz, yaw);
  send_msg_.position.x() = final_xyz.x();
  send_msg_.position.y() = final_xyz.y();
  send_msg_.position.z() = final_xyz.z();
  send_msg_.v_yaw = target_msg.v_yaw;
  send_msg_.pitch = pitch;
  send_msg_.yaw = yaw;
  metrics_msg_.is_fire = send_msg_.is_fire;

  publish_outputs();

  if ((frame_index_ % 30U) == 0U)
  {
    XR_LOG_INFO(
        "Aimer frame=%llu target=%s valid=%d converged=%d fire=%d iter=%u delay_s=%.3f fly_s=%.3f yaw=%.3f gimbal_yaw=%.3f cmd_err_deg=%.2f gimbal_err_deg=%.2f tol_deg=%.2f latency_ms=%.2f",
        static_cast<unsigned long long>(frame_index_),
        ArmorNumberToString(target_msg.id).data(), metrics_msg_.valid ? 1 : 0,
        metrics_msg_.converged ? 1 : 0, metrics_msg_.is_fire ? 1 : 0,
        metrics_msg_.iteration_count,
        metrics_msg_.delay_time_s, metrics_msg_.fly_time_s, metrics_msg_.yaw,
        last_fire_gimbal_yaw_rad_, last_fire_command_error_rad_ / DEG2RAD,
        last_fire_gimbal_error_rad_ / DEG2RAD, last_fire_tolerance_rad_ / DEG2RAD,
        metrics_msg_.latency_ms);
  }
}
