#pragma once

/**
 * @file AimerTargetModel.hpp
 * @brief tracker 目标预测和瞄点选择工具。
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "AimerMath.hpp"

namespace AimerDetail
{
/**
 * @brief 单个预测状态下选中的装甲板和策略元数据。
 */
struct AimPoint
{
  /// 该瞄点是否可用。
  bool valid{false};
  /// 该装甲板是否允许开火。
  bool shootable{false};
  /// 选中的装甲板面索引。
  int armor_index{0};
  /// 装甲板相对整车中心方位的视角，单位 rad。
  double view_angle{0.0};
  /// 选中装甲板中心 x、y、z 和装甲板 yaw。
  Eigen::Vector4d xyza{Eigen::Vector4d::Zero()};
};

/**
 * @brief 根据最近装甲板计算出的弹道命令。
 */
struct AimCommand
{
  /// yaw_pitch 和 fly_time 是否有效。
  bool valid{false};
  /// 命令 yaw 和 pitch，单位 rad。
  Eigen::Vector2d yaw_pitch{Eigen::Vector2d::Zero()};
  /// 估计弹丸飞行时间，单位 s。
  double fly_time{0.0};
  /// 用于计算命令的选中瞄点。
  AimPoint aim_point{};
};

/**
 * @brief 用于恒速预测的可变 tracker 目标快照。
 */
struct PredictedTarget
{
  /// 当前预测时刻的 tracker 目标状态。
  ArmorTrackerTarget msg{};

  /**
   * @brief 按时间增量推进目标中心和 yaw。
   * @param dt 预测时间增量，单位 s。
   */
  void Predict(double dt)
  {
    msg.position.x() += msg.velocity.x() * dt;
    msg.position.y() += msg.velocity.y() * dt;
    msg.position.z() += msg.velocity.z() * dt;
    msg.yaw = LimitRad(msg.yaw + msg.v_yaw * dt);
  }

  /**
   * @brief 将整车目标状态展开为装甲板中心和 yaw 候选。
   * @return 以 x、y、z、yaw 表示的装甲板候选列表。
   */
  std::vector<Eigen::Vector4d> GetArmorXYZAList() const
  {
    std::vector<Eigen::Vector4d> armor_xyza_list;
    armor_xyza_list.reserve(static_cast<std::size_t>(std::max(msg.armors_num, 0)));

    for (int index = 0; index < msg.armors_num; ++index)
    {
      const double angle = LimitRad(msg.yaw + index * 2.0 * PI / msg.armors_num);
      const bool use_length_height = (msg.armors_num == 4) && (index == 1 || index == 3);
      const double radius = use_length_height ? msg.radius_2 : msg.radius_1;
      // tracker 输出帧为 x 右、y 前、z 上；该展开式对应
      // ArmorTracker 内部 world frame 经 WorldToOutputFrame() 后的结果。
      const double armor_x = msg.position.x() + radius * std::sin(angle);
      const double armor_y = msg.position.y() - radius * std::cos(angle);
      const double armor_z =
          use_length_height ? msg.position.z() + msg.dz : msg.position.z();
      armor_xyza_list.push_back({armor_x, armor_y, armor_z, angle});
    }

    return armor_xyza_list;
  }
};

/**
 * @brief 计算弹丸飞行时间之前的固定预测延迟。
 * @param cfg Aimer 运行时配置。
 * @param target tracker 目标状态。
 * @return 固定预测延迟，单位 s。
 */
inline double FixedPredictDelay(const AimerConfig& cfg,
                                const ArmorTrackerTarget& target)
{
  const bool high_speed = std::abs(target.v_yaw) > cfg.yaw_rate_threshold;
  return cfg.image_to_now_s + cfg.vision_to_command_delay_s +
         cfg.command_transport_delay_s + cfg.gimbal_response_delay_s +
         (high_speed ? cfg.high_speed_extra_predict_s
                     : cfg.low_speed_extra_predict_s);
}

/**
 * @brief 计算装甲板相对整车中心方位的视角。
 * @param target tracker 目标状态。
 * @param xyza 装甲板中心和装甲板 yaw。
 * @return 视角，单位 rad。
 */
inline double ViewAngle(const ArmorTrackerTarget& target, const Eigen::Vector4d& xyza)
{
  const double center_yaw = BearingYaw(target.position);
  return LimitRad(xyza[3] - center_yaw);
}

/**
 * @brief 将选中的装甲板候选封装为 AimPoint。
 * @param target 预测后的 tracker 目标。
 * @param armor_index 选中的装甲板索引。
 * @param xyza 选中装甲板中心和装甲板 yaw。
 * @param shootable 该装甲板是否允许开火。
 * @return 填充后的瞄点。
 */
inline AimPoint BuildAimPoint(const PredictedTarget& target, int armor_index,
                              const Eigen::Vector4d& xyza, bool shootable)
{
  AimPoint out;
  out.valid = true;
  out.shootable = shootable;
  out.armor_index = armor_index;
  out.view_angle = ViewAngle(target.msg, xyza);
  out.xyza = xyza;
  return out;
}

/**
 * @brief 选择水平距离最近的装甲板候选。
 * @param target 预测后的 tracker 目标。
 * @param armor_xyza_list 候选装甲板中心和 yaw 列表。
 * @param lock_id 输入上一锁定索引，输出新的选中索引。
 * @return 选中的瞄点。
 */
inline AimPoint ChooseNearestArmor(
    const PredictedTarget& target, const std::vector<Eigen::Vector4d>& armor_xyza_list,
    int& lock_id)
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

  lock_id = nearest_index;
  return BuildAimPoint(target, nearest_index, armor_xyza_list[nearest_index], true);
}

/**
 * @brief 按水平距离最近原则选择瞄点。
 * @param target 预测后的 tracker 目标。
 * @param lock_id 输入上一锁定索引，输出新的选中索引。
 * @return 选中的瞄点；无可瞄目标时返回 invalid。
 */
inline AimPoint ChooseAimPoint(const PredictedTarget& target, int& lock_id)
{
  if (!target.msg.tracking)
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

  return ChooseNearestArmor(target, armor_xyza_list, lock_id);
}

/**
 * @brief 计算指向最近预测装甲板的 yaw、pitch 和飞行时间。
 * @param cfg Aimer 运行时配置。
 * @param target 预测后的 tracker 目标。
 * @param bullet_speed 弹速，单位 m/s。
 * @return 弹道解算成功时返回有效瞄准命令。
 */
inline AimCommand ComputeNearestAimCommand(const AimerConfig& cfg,
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
  command.aim_point = ChooseNearestArmor(target, armor_xyza_list, nearest_lock_id);

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
  command.yaw_pitch.y() = trajectory.pitch + cfg.pitch_offset * DEG2RAD;
  return command;
}
}  // namespace AimerDetail
