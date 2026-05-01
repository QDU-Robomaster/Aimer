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
constexpr double PLAN_DEFAULT_DT_S = 0.01;
constexpr double PLAN_MIN_DT_S = 0.001;
constexpr double PLAN_MAX_DT_S = 0.2;
constexpr double PLAN_MAX_RATE_RAD_S = 30.0;
constexpr double PLAN_MAX_ACC_RAD_S2 = 300.0;
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

double ClampSymmetric(double value, double limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

struct AimPoint
{
  bool valid{false};
  bool shootable{false};
  int armor_index{0};
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
      const double angle = LimitRad(msg.yaw + index * 2.0 * PI / msg.armors_num);
      const bool use_length_height = (msg.armors_num == 4) && (index == 1 || index == 3);
      const double radius = use_length_height ? msg.radius_2 : msg.radius_1;
      const double armor_x = msg.position.x() + radius * std::cos(angle);
      const double armor_y = msg.position.y() + radius * std::sin(angle);
      const double armor_z =
          use_length_height ? msg.position.z() + msg.dz : msg.position.z();
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
  return {true, true, nearest_index, armor_xyza_list[nearest_index]};
}

AimPoint ChooseRotatingArmor(const Aimer::Config& cfg, const PredictedTarget& target,
                             const std::vector<Eigen::Vector4d>& armor_xyza_list,
                             int& lock_id)
{
  const bool is_outpost = target.msg.id == ArmorNumber::OUTPOST;
  const double center_yaw = std::atan2(target.msg.position.y(), target.msg.position.x());
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
      return {true, true, index, armor_xyza_list[index]};
    }
  }

  if (lock_id >= 0 && lock_id < static_cast<int>(armor_xyza_list.size()))
  {
    return {true, false, lock_id, armor_xyza_list[lock_id]};
  }

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
  return {true, false, nearest_index, armor_xyza_list[nearest_index]};
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
  command.aim_point = ChooseNearestArmor(armor_xyza_list, nearest_lock_id);

  const Eigen::Vector3d xyz = command.aim_point.xyza.head<3>();
  const double horizontal_distance = std::hypot(xyz.x(), xyz.y());
  const auto trajectory = SolveTrajectoryPitch(bullet_speed, horizontal_distance, xyz.z());
  if (trajectory.unsolvable)
  {
    return command;
  }

  command.valid = true;
  command.fly_time = trajectory.fly_time;
  command.yaw_pitch.x() =
      LimitRad(std::atan2(xyz.y(), xyz.x()) + cfg.yaw_offset * DEG2RAD);
  command.yaw_pitch.y() = -(trajectory.pitch + cfg.pitch_offset * DEG2RAD);
  return command;
}

bool BuildReferenceTrajectory(const Aimer::Config& cfg,
                              const SolveTrajectory::Target& target_msg,
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
        auto* gimbal_rotation_msg =
            reinterpret_cast<LibXR::Quaternion<float>*>(data.addr_);
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
  gimbal_rotation_ =
      LibXR::Quaternion<double>(gimbal_rotation_msg.w(), gimbal_rotation_msg.x(),
                                gimbal_rotation_msg.y(), gimbal_rotation_msg.z());
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
      (target_distance > cfg_.judge_distance ? cfg_.second_tolerance
                                             : cfg_.first_tolerance) *
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

void Aimer::BuildTrajectoryMessage(const SolveTrajectory::Target& target_msg,
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

  const double horizontal_distance = std::hypot(aim_point.x(), aim_point.y());
  if (fly_time <= 0.0 || bullet_speed <= 0.0 ||
      horizontal_distance <= MIN_HORIZONTAL_DISTANCE_M)
  {
    trajectory_msg_.valid = false;
    return;
  }

  const double dir_x = aim_point.x() / horizontal_distance;
  const double dir_y = aim_point.y() / horizontal_distance;
  const double v_horizontal = bullet_speed * std::cos(launch_pitch);
  const double v_vertical = bullet_speed * std::sin(launch_pitch);
  trajectory_msg_.point_count = AimerTrajectory::MAX_POINTS;
  for (uint8_t index = 0; index < AimerTrajectory::MAX_POINTS; ++index)
  {
    const double ratio = static_cast<double>(index) /
                         static_cast<double>(AimerTrajectory::MAX_POINTS - 1U);
    const double t = fly_time * ratio;
    const double s = v_horizontal * t;
    const double z = v_vertical * t - 0.5 * GRAVITY * t * t;
    trajectory_msg_.points[index] = LibXR::Position<double>(dir_x * s, dir_y * s, z);
  }
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
  has_last_plan_command_ = false;
  has_last_plan_velocity_ = false;
  last_plan_timestamp_us_ = 0;
  last_plan_yaw_ = 0.0;
  last_plan_pitch_ = 0.0;
  last_plan_yaw_vel_ = 0.0;
  last_plan_pitch_vel_ = 0.0;
  last_plan_mpc_ = false;
}

bool Aimer::BuildMpcGimbalPlan(const SolveTrajectory::Target& target_msg,
                               double bullet_speed, bool)
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
  gimbal_plan_msg_.fire = plan_error < cfg_.mpc_fire_thresh;
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

void Aimer::BuildFiniteDifferenceGimbalPlan(const SolveTrajectory::Target& target_msg,
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

  double dt_s = PLAN_DEFAULT_DT_S;
  bool have_derivative_dt = false;
  if (has_last_plan_command_)
  {
    if (target_msg.image_timestamp_us > last_plan_timestamp_us_ &&
        last_plan_timestamp_us_ != 0)
    {
      dt_s = static_cast<double>(target_msg.image_timestamp_us - last_plan_timestamp_us_) *
             1e-6;
    }
    have_derivative_dt = dt_s >= PLAN_MIN_DT_S && dt_s <= PLAN_MAX_DT_S;
  }

  double yaw_vel = 0.0;
  double pitch_vel = 0.0;
  double yaw_acc = 0.0;
  double pitch_acc = 0.0;
  if (have_derivative_dt)
  {
    yaw_vel =
        ClampSymmetric(LimitRad(yaw - last_plan_yaw_) / dt_s, PLAN_MAX_RATE_RAD_S);
    pitch_vel = ClampSymmetric(LimitRad(pitch - last_plan_pitch_) / dt_s,
                               PLAN_MAX_RATE_RAD_S);
    if (has_last_plan_velocity_)
    {
      yaw_acc =
          ClampSymmetric((yaw_vel - last_plan_yaw_vel_) / dt_s, PLAN_MAX_ACC_RAD_S2);
      pitch_acc = ClampSymmetric((pitch_vel - last_plan_pitch_vel_) / dt_s,
                                 PLAN_MAX_ACC_RAD_S2);
    }
  }

  gimbal_plan_msg_.yaw_vel = static_cast<float>(yaw_vel);
  gimbal_plan_msg_.pitch_vel = static_cast<float>(pitch_vel);
  gimbal_plan_msg_.yaw_acc = static_cast<float>(yaw_acc);
  gimbal_plan_msg_.pitch_acc = static_cast<float>(pitch_acc);

  has_last_plan_command_ = true;
  has_last_plan_velocity_ = have_derivative_dt;
  last_plan_timestamp_us_ = target_msg.image_timestamp_us;
  last_plan_yaw_ = yaw;
  last_plan_pitch_ = pitch;
  last_plan_yaw_vel_ = yaw_vel;
  last_plan_pitch_vel_ = pitch_vel;
}

void Aimer::BuildGimbalPlan(const SolveTrajectory::Target& target_msg, bool control,
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

void Aimer::TargetCallback(const SolveTrajectory::Target& target_msg)
{
  const auto start_time = std::chrono::steady_clock::now();
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
  };

  ++frame_index_;
  metrics_msg_ = {};
  metrics_msg_.frame_index = frame_index_;
  metrics_msg_.target_tracking = target_msg.tracking;
  metrics_msg_.target_id = target_msg.id;
  trajectory_msg_ = {};
  trajectory_msg_.image_timestamp_us = target_msg.image_timestamp_us;
  trajectory_msg_.target_id = target_msg.id;
  gimbal_plan_msg_ = {};
  gimbal_plan_msg_.image_timestamp_us = target_msg.image_timestamp_us;

  if (target_msg.id != last_target_id_)
  {
    lock_id_ = -1;
    has_last_command_ = false;
    last_target_id_ = target_msg.id;
    ResetGimbalPlanHistory();
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
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  const double delay_time = (std::abs(target_msg.v_yaw) > cfg_.yaw_rate_threshold)
                                ? cfg_.high_speed_delay_time
                                : cfg_.low_speed_delay_time;
  metrics_msg_.delay_time_s = delay_time;

  PredictedTarget base_target{target_msg};
  base_target.Predict(delay_time);

  AimPoint debug_aim_point = ChooseAimPoint(cfg_, base_target, lock_id_);

  if (!debug_aim_point.valid)
  {
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  const Eigen::Vector3d xyz_0 = debug_aim_point.xyza.head<3>();
  const double horizontal_distance_0 = std::hypot(xyz_0.x(), xyz_0.y());
  auto trajectory = SolveTrajectoryPitch(bullet_speed, horizontal_distance_0, xyz_0.z());

  if (trajectory.unsolvable)
  {
    ResetGimbalPlanHistory();
    publish_outputs(false);
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
    ResetGimbalPlanHistory();
    publish_outputs(false);
    return;
  }

  metrics_msg_.valid = true;
  metrics_msg_.converged = converged;
  metrics_msg_.fly_time_s = trajectory.fly_time;

  const Eigen::Vector3d final_xyz = debug_aim_point.xyza.head<3>();
  const double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + cfg_.yaw_offset * DEG2RAD;
  const double pitch = -(trajectory.pitch + cfg_.pitch_offset * DEG2RAD);

  metrics_msg_.yaw = yaw;
  metrics_msg_.pitch = pitch;

  target_euler_msg_.Pitch() = static_cast<float>(pitch);
  target_euler_msg_.Yaw() = static_cast<float>(yaw);

  send_msg_.is_fire = debug_aim_point.shootable && ShouldAutoFire(final_xyz, yaw);
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
  BuildGimbalPlan(target_msg, true, send_msg_.is_fire, yaw, pitch, bullet_speed);

  publish_outputs(true);

  if ((frame_index_ % 30U) == 0U)
  {
    XR_LOG_INFO(
        "Aimer frame=%llu target=%s valid=%d converged=%d fire=%d iter=%u delay_s=%.3f "
        "fly_s=%.3f yaw=%.3f gimbal_yaw=%.3f cmd_err_deg=%.2f gimbal_err_deg=%.2f "
        "tol_deg=%.2f plan_mpc=%d plan_yaw=%.3f plan_pitch=%.3f "
        "plan_yaw_vel=%.3f plan_pitch_vel=%.3f plan_yaw_acc=%.3f "
        "plan_pitch_acc=%.3f latency_ms=%.2f",
        static_cast<unsigned long long>(frame_index_),
        ArmorNumberToString(target_msg.id).data(), metrics_msg_.valid ? 1 : 0,
        metrics_msg_.converged ? 1 : 0, metrics_msg_.is_fire ? 1 : 0,
        metrics_msg_.iteration_count, metrics_msg_.delay_time_s, metrics_msg_.fly_time_s,
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
