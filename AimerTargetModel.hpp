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
/// 普通四面目标的正面可打窗口。
inline constexpr double NORMAL_FACE_WINDOW_RAD = 60.0 * DEG2RAD;
/// 普通旋转目标进入可打区的角度窗口。
inline constexpr double NORMAL_APPROACH_WINDOW_RAD = 55.0 * DEG2RAD;
/// 普通旋转目标离开可打区的角度窗口。
inline constexpr double NORMAL_EXIT_WINDOW_RAD = 20.0 * DEG2RAD;
/// 前哨站进入可打区的角度窗口。
inline constexpr double OUTPOST_APPROACH_WINDOW_RAD = 70.0 * DEG2RAD;
/// 前哨站离开可打区的角度窗口。
inline constexpr double OUTPOST_EXIT_WINDOW_RAD = 30.0 * DEG2RAD;

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
 * @brief 根据策略选中装甲板计算出的弹道命令。
 */
struct AimCommand
{
  /// yaw_roll 和 fly_time 是否有效。
  bool valid{false};
  /// 命令 yaw 和机械 roll 轴，单位 rad。
  Eigen::Vector2d yaw_roll{Eigen::Vector2d::Zero()};
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
      // tracker 输出帧为 x 右、y 前、z 上。
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
 * @brief 判断目标是否按低速旋转策略处理。
 * @param cfg Aimer 运行时配置。
 * @param target tracker 目标状态。
 * @return yaw 角速度未超过策略阈值时返回 true。
 */
inline bool IsLowYawRateTarget(const AimerConfig& cfg,
                               const ArmorTrackerTarget& target)
{
  return std::abs(target.v_yaw) <= cfg.yaw_rate_threshold;
}

/**
 * @brief 判断当前选中的装甲板姿态是否允许开火。
 * @param cfg Aimer 运行时配置。
 * @param target tracker 目标状态。
 * @param view_angle 装甲板相对整车中心方位的视角，单位 rad。
 * @return 该姿态处于可打窗口时返回 true。
 */
inline bool IsArmorFaceShootable(const AimerConfig& cfg,
                                 const ArmorTrackerTarget& target,
                                 double view_angle)
{
  if (target.id == ArmorNumber::OUTPOST)
  {
    return std::abs(view_angle) <= OUTPOST_APPROACH_WINDOW_RAD;
  }

  if (IsLowYawRateTarget(cfg, target))
  {
    return std::abs(view_angle) <= NORMAL_FACE_WINDOW_RAD;
  }

  if (std::abs(view_angle) > NORMAL_APPROACH_WINDOW_RAD)
  {
    return false;
  }
  return target.v_yaw > 0.0 ? view_angle < NORMAL_EXIT_WINDOW_RAD
                            : view_angle > -NORMAL_EXIT_WINDOW_RAD;
}

/**
 * @brief 将面索引限制到当前目标有效范围。
 * @param face_index 输入面索引。
 * @param face_count 当前目标面数。
 * @return 有效面索引。
 */
inline int ClampFaceIndex(int face_index, int face_count)
{
  return std::clamp(face_index, 0, std::max(1, face_count) - 1);
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
 * @brief 按指定索引选择装甲板候选。
 * @param cfg Aimer 运行时配置。
 * @param target 预测后的 tracker 目标。
 * @param armor_xyza_list 候选装甲板中心和 yaw 列表。
 * @param armor_index 指定面索引。
 * @param lock_id 输出新的锁定索引。
 * @return 选中的瞄点。
 */
inline AimPoint ChooseArmorByIndex(
    const AimerConfig& cfg, const PredictedTarget& target,
    const std::vector<Eigen::Vector4d>& armor_xyza_list, int armor_index,
    int& lock_id)
{
  if (armor_xyza_list.empty())
  {
    lock_id = -1;
    return {};
  }

  const int selected_index =
      ClampFaceIndex(armor_index, static_cast<int>(armor_xyza_list.size()));
  lock_id = selected_index;
  const double view_angle = ViewAngle(target.msg, armor_xyza_list[selected_index]);
  return BuildAimPoint(target, selected_index, armor_xyza_list[selected_index],
                       IsArmorFaceShootable(cfg, target.msg, view_angle));
}

/**
 * @brief 选择水平距离最近的装甲板索引。
 */
inline int NearestArmorIndex(const std::vector<Eigen::Vector4d>& armor_xyza_list)
{
  if (armor_xyza_list.empty())
  {
    return -1;
  }

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

  return nearest_index;
}

/**
 * @brief 选择水平距离最近的装甲板候选。
 * @param target 预测后的 tracker 目标。
 * @param armor_xyza_list 候选装甲板中心和 yaw 列表。
 * @param lock_id 输入上一锁定索引，输出新的选中索引。
 * @param shootable 该选择是否允许开火。
 * @return 选中的瞄点。
 */
inline AimPoint ChooseNearestArmor(
    const PredictedTarget& target, const std::vector<Eigen::Vector4d>& armor_xyza_list,
    int& lock_id, bool shootable)
{
  const int nearest_index = NearestArmorIndex(armor_xyza_list);
  if (nearest_index < 0)
  {
    lock_id = -1;
    return {};
  }

  lock_id = nearest_index;
  return BuildAimPoint(target, nearest_index, armor_xyza_list[nearest_index],
                       shootable);
}

/**
 * @brief 为轨迹规划选择水平距离最近的装甲板，并重新计算命中姿态。
 * @param cfg Aimer 运行时配置。
 * @param target 预测后的 tracker 目标。
 * @param armor_xyza_list 候选装甲板中心和 yaw 列表。
 * @param lock_id 输出新的选中索引。
 * @return 选中的轨迹参考瞄点。
 */
inline AimPoint ChooseNearestTrajectoryArmor(
    const AimerConfig& cfg, const PredictedTarget& target,
    const std::vector<Eigen::Vector4d>& armor_xyza_list, int& lock_id)
{
  const int nearest_index = NearestArmorIndex(armor_xyza_list);
  if (nearest_index < 0)
  {
    lock_id = -1;
    return {};
  }
  return ChooseArmorByIndex(cfg, target, armor_xyza_list, nearest_index, lock_id);
}

/**
 * @brief 计算每个候选面相对相机方向的角度。
 * @param target 预测后的 tracker 目标。
 * @param armor_xyza_list 候选装甲板中心和 yaw 列表。
 * @return 每个候选面的相对视角，单位 rad。
 */
inline std::vector<double> BuildFaceViewAngles(
    const PredictedTarget& target,
    const std::vector<Eigen::Vector4d>& armor_xyza_list)
{
  std::vector<double> view_angles;
  view_angles.reserve(armor_xyza_list.size());
  for (const auto& armor_xyza : armor_xyza_list)
  {
    view_angles.emplace_back(ViewAngle(target.msg, armor_xyza));
  }
  return view_angles;
}

/**
 * @brief 在正面可打窗口中按锁定滞回选择装甲板。
 * @param target 预测后的 tracker 目标。
 * @param armor_xyza_list 候选装甲板中心和 yaw 列表。
 * @param view_angles 候选面相对相机方向的角度。
 * @param lock_id 输入上一锁定索引，输出新的锁定索引。
 * @return 选中的瞄点；窗口内无候选时返回 invalid。
 */
inline AimPoint ChooseWindowLockedArmor(
    const PredictedTarget& target, const std::vector<Eigen::Vector4d>& armor_xyza_list,
    const std::vector<double>& view_angles, int& lock_id)
{
  std::vector<int> candidates;
  candidates.reserve(armor_xyza_list.size());
  for (int index = 0; index < static_cast<int>(armor_xyza_list.size()); ++index)
  {
    if (std::abs(view_angles[index]) <= NORMAL_FACE_WINDOW_RAD)
    {
      candidates.push_back(index);
    }
  }

  if (candidates.empty())
  {
    return {};
  }

  if (candidates.size() > 1)
  {
    const int first = candidates[0];
    const int second = candidates[1];
    if (lock_id != first && lock_id != second)
    {
      lock_id = (std::abs(view_angles[first]) < std::abs(view_angles[second]))
                     ? first
                     : second;
    }
    return BuildAimPoint(target, lock_id, armor_xyza_list[lock_id], true);
  }

  lock_id = -1;
  const int selected_index = candidates[0];
  return BuildAimPoint(target, selected_index, armor_xyza_list[selected_index], true);
}

/**
 * @brief 对高速旋转或前哨站目标按进入/离开窗口选择装甲板。
 * @param target 预测后的 tracker 目标。
 * @param armor_xyza_list 候选装甲板中心和 yaw 列表。
 * @param view_angles 候选面相对相机方向的角度。
 * @param lock_id 输入上一锁定索引，输出新的锁定索引。
 * @return 选中的瞄点；当前锁定面不可打时 shootable 为 false。
 */
inline AimPoint ChooseDirectionalArmor(
    const PredictedTarget& target, const std::vector<Eigen::Vector4d>& armor_xyza_list,
    const std::vector<double>& view_angles, int& lock_id)
{
  const bool is_outpost = target.msg.id == ArmorNumber::OUTPOST;
  const double approach_window =
      is_outpost ? OUTPOST_APPROACH_WINDOW_RAD : NORMAL_APPROACH_WINDOW_RAD;
  const double exit_window =
      is_outpost ? OUTPOST_EXIT_WINDOW_RAD : NORMAL_EXIT_WINDOW_RAD;

  for (int index = 0; index < static_cast<int>(armor_xyza_list.size()); ++index)
  {
    const double view_angle = view_angles[index];
    if (std::abs(view_angle) > approach_window)
    {
      continue;
    }

    if ((target.msg.v_yaw > 0.0 && view_angle < exit_window) ||
        (target.msg.v_yaw < 0.0 && view_angle > -exit_window))
    {
      lock_id = index;
      return BuildAimPoint(target, index, armor_xyza_list[index], true);
    }
  }

  if (lock_id >= 0 && lock_id < static_cast<int>(armor_xyza_list.size()))
  {
    return BuildAimPoint(target, lock_id, armor_xyza_list[lock_id], false);
  }

  return ChooseNearestArmor(target, armor_xyza_list, lock_id, false);
}

/**
 * @brief 按 tracker 面状态和可打窗口选择瞄点。
 * @param cfg Aimer 运行时配置。
 * @param target 预测后的 tracker 目标。
 * @param lock_id 输入上一锁定索引，输出新的选中索引。
 * @return 选中的瞄点；无可瞄目标时返回 invalid。
 */
inline AimPoint ChooseAimPoint(const AimerConfig& cfg, const PredictedTarget& target,
                               int& lock_id)
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

  if (!target.msg.face_switch_observed)
  {
    return ChooseArmorByIndex(cfg, target, armor_xyza_list,
                              target.msg.tracked_face_index, lock_id);
  }

  const auto view_angles = BuildFaceViewAngles(target, armor_xyza_list);
  const bool is_outpost = target.msg.id == ArmorNumber::OUTPOST;
  if (!is_outpost && IsLowYawRateTarget(cfg, target.msg))
  {
    return ChooseWindowLockedArmor(target, armor_xyza_list, view_angles, lock_id);
  }

  return ChooseDirectionalArmor(target, armor_xyza_list, view_angles, lock_id);
}

/**
 * @brief 轨迹规划专用瞄点选择。
 *
 * 该选择器不使用 tracker 当前绑定面的硬锁，而是在每个预测采样点重新选择
 * 几何上最近的装甲板，使 MPC reference 能提前看到未来切板。
 */
inline AimPoint ChooseTrajectoryAimPoint(const AimerConfig& cfg,
                                         const PredictedTarget& target,
                                         int& lock_id)
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

  return ChooseNearestTrajectoryArmor(cfg, target, armor_xyza_list, lock_id);
}

/**
 * @brief 由有效瞄准命令生成单发命中候选。
 */
inline AimerShotCandidate MakeShotCandidate(const AimPoint& aim_point, double yaw,
                                            double roll, double fly_time)
{
  AimerShotCandidate candidate;
  if (!aim_point.valid)
  {
    return candidate;
  }

  candidate.valid = true;
  candidate.face_shootable_at_hit = aim_point.shootable;
  candidate.hit_face = aim_point.armor_index;
  candidate.view_angle = aim_point.view_angle;
  candidate.hit_xyza = aim_point.xyza;
  candidate.yaw = yaw;
  candidate.roll = roll;
  candidate.fly_time = fly_time;
  return candidate;
}

/**
 * @brief 由有效瞄准命令生成单发命中候选。
 */
inline AimerShotCandidate MakeShotCandidate(const AimCommand& command)
{
  if (!command.valid)
  {
    return {};
  }
  return MakeShotCandidate(command.aim_point, command.yaw_roll.x(),
                           command.yaw_roll.y(), command.fly_time);
}

/**
 * @brief 根据已经选好的瞄点解算 yaw、roll 轴命令和飞行时间。
 */
inline AimCommand BuildAimCommandFromAimPoint(const AimerConfig& cfg,
                                             const AimPoint& aim_point,
                                             double bullet_speed)
{
  AimCommand command;
  command.aim_point = aim_point;
  if (!command.aim_point.valid)
  {
    return command;
  }

  const Eigen::Vector3d xyz = command.aim_point.xyza.head<3>();
  const double horizontal_distance = HorizontalDistance(xyz);
  const auto trajectory =
      SolveTrajectoryElevation(bullet_speed, horizontal_distance, BallisticHeight(xyz));
  if (trajectory.unsolvable)
  {
    return command;
  }

  command.valid = true;
  command.fly_time = trajectory.fly_time;
  command.yaw_roll.x() =
      LimitRad(BearingYaw(xyz) + cfg.yaw_offset * DEG2RAD);
  command.yaw_roll.y() = trajectory.elevation + cfg.roll_offset * DEG2RAD;
  return command;
}

/**
 * @brief 判断弹道射线是否需要穿过车体才能打到该装甲板。
 */
inline bool IsArmorOnVisibleSide(const PredictedTarget& target,
                                 const Eigen::Vector4d& xyza)
{
  const Eigen::Vector2d center_to_camera(-target.msg.position.x(),
                                         -target.msg.position.y());
  const Eigen::Vector2d center_to_armor(
      xyza.x() - target.msg.position.x(), xyza.y() - target.msg.position.y());
  const double norm_product =
      center_to_camera.norm() * center_to_armor.norm();
  if (norm_product <= MIN_HORIZONTAL_DISTANCE_M)
  {
    return true;
  }

  return center_to_camera.dot(center_to_armor) > 0.0;
}

/**
 * @brief 按给定枪线在所有物理装甲面中选择最可能命中的候选。
 */
inline AimerShotCandidate ChooseShotCandidateForCommand(
    const AimerConfig& cfg, const PredictedTarget& target, double bullet_speed,
    double command_yaw, double command_roll)
{
  AimerShotCandidate best_candidate;
  double best_error = std::numeric_limits<double>::max();

  if (!target.msg.tracking)
  {
    return best_candidate;
  }

  const auto armor_xyza_list = target.GetArmorXYZAList();
  for (int index = 0; index < static_cast<int>(armor_xyza_list.size()); ++index)
  {
    const auto& armor_xyza = armor_xyza_list[static_cast<std::size_t>(index)];
    const double view_angle = ViewAngle(target.msg, armor_xyza);
    if (!IsArmorFaceShootable(cfg, target.msg, view_angle) ||
        !IsArmorOnVisibleSide(target, armor_xyza))
    {
      continue;
    }

    const AimPoint aim_point = BuildAimPoint(target, index, armor_xyza, true);
    const AimCommand command =
        BuildAimCommandFromAimPoint(cfg, aim_point, bullet_speed);
    if (!command.valid)
    {
      continue;
    }

    const double error =
        std::hypot(LimitRad(command.yaw_roll.x() - command_yaw),
                   command.yaw_roll.y() - command_roll);
    if (error < best_error)
    {
      best_error = error;
      best_candidate = MakeShotCandidate(command);
    }
  }

  return best_candidate;
}

/**
 * @brief 计算指向策略选中装甲板的 yaw、roll 轴命令和飞行时间。
 * @param cfg Aimer 运行时配置。
 * @param target 预测后的 tracker 目标。
 * @param bullet_speed 弹速，单位 m/s。
 * @param lock_id 输入上一锁定索引，输出新的锁定索引。
 * @return 弹道解算成功时返回有效瞄准命令。
 */
inline AimCommand ComputeAimCommand(const AimerConfig& cfg,
                                    const PredictedTarget& target,
                                    double bullet_speed, int& lock_id)
{
  return BuildAimCommandFromAimPoint(
      cfg, ChooseAimPoint(cfg, target, lock_id), bullet_speed);
}

/**
 * @brief 计算轨迹规划 reference 使用的 yaw、roll 轴命令和飞行时间。
 */
inline AimCommand ComputeTrajectoryAimCommand(const AimerConfig& cfg,
                                             const PredictedTarget& target,
                                             double bullet_speed, int& lock_id)
{
  return BuildAimCommandFromAimPoint(
      cfg, ChooseTrajectoryAimPoint(cfg, target, lock_id), bullet_speed);
}
}  // namespace AimerDetail
