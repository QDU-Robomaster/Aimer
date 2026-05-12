#pragma once

/**
 * @file AimerTargetModel.hpp
 * @brief Tracker target prediction and aim-point selection helpers.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "AimerMath.hpp"

namespace AimerDetail
{
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

inline Aimer::Strategy SelectStrategy(const Aimer::Config& cfg,
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

inline double FixedPredictDelay(const Aimer::Config& cfg,
                                const ArmorTrackerTarget& target)
{
  const bool high_speed = std::abs(target.v_yaw) > cfg.yaw_rate_threshold;
  return cfg.image_to_now_s + cfg.vision_to_command_delay_s +
         cfg.command_transport_delay_s + cfg.gimbal_response_delay_s +
         (high_speed ? cfg.high_speed_extra_predict_s
                     : cfg.low_speed_extra_predict_s);
}

inline double ViewAngle(const ArmorTrackerTarget& target, const Eigen::Vector4d& xyza)
{
  const double center_yaw = BearingYaw(target.position);
  return LimitRad(xyza[3] - center_yaw);
}

inline bool IsFrontFacing(const Aimer::Config& cfg, const ArmorTrackerTarget& target,
                          const Eigen::Vector4d& xyza)
{
  (void)cfg;
  return std::abs(ViewAngle(target, xyza)) <= 75.0 * DEG2RAD;
}

inline AimPoint BuildAimPoint(const Aimer::Config& cfg, const PredictedTarget& target,
                              int armor_index, const Eigen::Vector4d& xyza,
                              Aimer::Strategy strategy,
                              Aimer::SelectReason reason,
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

inline AimPoint ChooseNearestArmor(
    const Aimer::Config& cfg, const PredictedTarget& target,
    const std::vector<Eigen::Vector4d>& armor_xyza_list, int& lock_id,
    Aimer::Strategy strategy)
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

inline AimPoint ChooseAimPoint(const Aimer::Config& cfg, const PredictedTarget& target,
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

inline AimCommand ComputeNearestAimCommand(const Aimer::Config& cfg,
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
}  // namespace AimerDetail
