#pragma once

/**
 * @file AimerTargetModel.hpp
 * @brief tracker 目标预测和瞄点选择工具。
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
/**
 * @brief 单个预测状态下选中的装甲板和策略元数据。
 */
struct AimPoint
{
  /// 该瞄点是否可用。
  bool valid{false};
  /// 该装甲板是否允许开火。
  bool shootable{false};
  /// 该装甲板是否满足正面角度门控。
  bool front_facing{false};
  /// 参与选择的候选装甲板数量。
  uint8_t candidate_count{0};
  /// 选中的装甲板面索引。
  int armor_index{0};
  /// 装甲板相对整车中心方位的视角，单位 rad。
  double view_angle{0.0};
  /// 当前选择策略。
  Aimer::Strategy strategy{Aimer::Strategy::LOST};
  /// 装甲板被选中的原因。
  Aimer::SelectReason selected_reason{Aimer::SelectReason::NONE};
  /// 选中装甲板发生变化的原因。
  Aimer::SwitchReason switch_reason{Aimer::SwitchReason::NONE};
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
      const double armor_x = msg.position.x() + radius * std::cos(angle);
      const double armor_y =
          use_length_height ? msg.position.y() + msg.dz : msg.position.y();
      const double armor_z = msg.position.z() + radius * std::sin(angle);
      armor_xyza_list.push_back({armor_x, armor_y, armor_z, angle});
    }

    return armor_xyza_list;
  }
};

/**
 * @brief 将 tracker 目标分类为当前 Aimer 策略。
 * @param cfg Aimer 运行时配置。
 * @param target tracker 目标状态。
 * @return 目标当前策略。
 */
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

/**
 * @brief 计算弹丸飞行时间之前的固定预测延迟。
 * @param cfg Aimer 运行时配置。
 * @param target tracker 目标状态。
 * @return 固定预测延迟，单位 s。
 */
inline double FixedPredictDelay(const Aimer::Config& cfg,
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
 * @brief 检查装甲板是否满足当前正面角度门控。
 * @param cfg Aimer 运行时配置。
 * @param target tracker 目标状态。
 * @param xyza 装甲板中心和装甲板 yaw。
 * @return 装甲板位于正面角度窗口内时返回 true。
 */
inline bool IsFrontFacing(const Aimer::Config& cfg, const ArmorTrackerTarget& target,
                          const Eigen::Vector4d& xyza)
{
  (void)cfg;
  return std::abs(ViewAngle(target, xyza)) <= 75.0 * DEG2RAD;
}

/**
 * @brief 将选中的装甲板候选封装为 AimPoint。
 * @param cfg Aimer 运行时配置。
 * @param target 预测后的 tracker 目标。
 * @param armor_index 选中的装甲板索引。
 * @param xyza 选中装甲板中心和装甲板 yaw。
 * @param strategy 当前选择策略。
 * @param reason 选择原因。
 * @param switch_reason 锁定切换原因。
 * @param shootable 该装甲板是否允许开火。
 * @param candidate_count 候选装甲板数量。
 * @return 填充后的瞄点。
 */
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

/**
 * @brief 选择水平距离最近的装甲板候选。
 * @param cfg Aimer 运行时配置。
 * @param target 预测后的 tracker 目标。
 * @param armor_xyza_list 候选装甲板中心和 yaw 列表。
 * @param lock_id 输入上一锁定索引，输出新的选中索引。
 * @param strategy 附加到结果上的当前策略。
 * @return 选中的瞄点。
 */
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

/**
 * @brief 按当前目标策略选择瞄点。
 * @param cfg Aimer 运行时配置。
 * @param target 预测后的 tracker 目标。
 * @param lock_id 输入上一锁定索引，输出新的选中索引。
 * @return 选中的瞄点；无可瞄目标时返回 invalid。
 */
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

/**
 * @brief 计算指向最近预测装甲板的 yaw、pitch 和飞行时间。
 * @param cfg Aimer 运行时配置。
 * @param target 预测后的 tracker 目标。
 * @param bullet_speed 弹速，单位 m/s。
 * @return 弹道解算成功时返回有效瞄准命令。
 */
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
