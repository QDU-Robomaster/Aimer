#pragma once

/**
 * @file AimerMath.hpp
 * @brief Aimer 实现使用的角度和弹道数学工具。
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Eigen/Dense>

namespace AimerDetail
{
/// 角度工具使用的圆周率常量。
inline constexpr double PI = 3.14159265358979323846;
/// 角度转弧度系数。
inline constexpr double DEG2RAD = PI / 180.0;
/// 弹道模型使用的重力加速度，单位 m/s^2。
inline constexpr double GRAVITY = 9.7833;
/// 避免弹道方程奇异的最小水平距离。
inline constexpr double MIN_HORIZONTAL_DISTANCE_M = 1e-4;
/// RoboMaster 小装甲板规则宽度，单位 m。
inline constexpr double SMALL_ARMOR_WIDTH_M = 0.135;
/// RoboMaster 装甲板规则高度，单位 m。
inline constexpr double ARMOR_HEIGHT_M = 0.056;
/// 自动开火角阈值预留的弹道散布裕量，单位 m。
inline constexpr double FIRE_BULLET_SPREAD_M = 0.015;
/// RoboMaster 装甲板固定安装俯仰角，单位 deg。
inline constexpr double ARMOR_PITCH_DEG = 15.0;
/// TinyMPC ADMM 最大迭代次数。
inline constexpr int MPC_MAX_ITER = 10;

/**
 * @brief 将角度限制到 [-pi, pi] 区间。
 * @param angle 输入角度，单位 rad。
 * @return 归一化后的角度，单位 rad。
 */
inline double LimitRad(double angle)
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

/**
 * @brief 计算 tracker x-y 水平面距离。
 * @tparam Derived Eigen 向量或表达式类型。
 * @param point 使用其 x 和 y 分量的点。
 * @return 水平距离，单位 m。
 */
template <typename Derived>
inline double HorizontalDistance(const Eigen::MatrixBase<Derived>& point)
{
  return std::hypot(point.x(), point.y());
}

/**
 * @brief 计算 tracker x 右、y 前水平面中的 yaw 方位角。
 * @tparam Derived Eigen 向量或表达式类型。
 * @param point 使用其 x 和 y 分量的点。
 * @return yaw 方位角，单位 rad。
 */
template <typename Derived>
inline double BearingYaw(const Eigen::MatrixBase<Derived>& point)
{
  return std::atan2(-point.x(), point.y());
}

/**
 * @brief 从 tracker 坐标点中取出弹道高度坐标。
 * @tparam Derived Eigen 向量或表达式类型。
 * @param point 使用其 z 分量作为高度的点。
 * @return 高度，单位 m。
 */
template <typename Derived>
inline double BallisticHeight(const Eigen::MatrixBase<Derived>& point)
{
  return point.z();
}

/**
 * @brief 闭式弹道 pitch 解算结果。
 */
struct TrajectorySolution
{
  /// 是否不存在有效弹道解。
  bool unsolvable{false};
  /// 估计弹丸飞行时间，单位 s。
  double fly_time{0.0};
  /// 加配置 pitch 偏置前的发射 pitch，单位 rad。
  double pitch{0.0};
};

/**
 * @brief 为选中装甲板计算动态 yaw 开火阈值。
 * @param cfg Aimer 运行时配置。
 * @param target_xyz tracker 输出 B 坐标系下的选中瞄点。
 * @param selected_view_angle 装甲板相对整车中心方位的视角。
 * @return yaw 阈值，单位 rad。
 */
inline double DynamicYawFireThreshold(const AimerConfig& cfg,
                                      const Eigen::Vector3d& target_xyz,
                                      double selected_view_angle)
{
  const double horizontal_distance =
      std::max(MIN_HORIZONTAL_DISTANCE_M, HorizontalDistance(target_xyz));
  const double facing_scale =
      std::clamp(std::cos(std::abs(selected_view_angle)), 0.25, 1.0);
  const double yaw_half =
      std::atan2(0.5 * SMALL_ARMOR_WIDTH_M * facing_scale, horizontal_distance);
  const double spread_yaw = std::atan2(FIRE_BULLET_SPREAD_M, horizontal_distance);
  return std::clamp(yaw_half - spread_yaw, cfg.min_fire_threshold,
                    cfg.max_fire_threshold);
}

/**
 * @brief 为选中装甲板计算动态 pitch/roll 开火阈值。
 * @param cfg Aimer 运行时配置。
 * @param target_xyz tracker 输出 B 坐标系下的选中瞄点。
 * @return pitch/roll 阈值，单位 rad。
 */
inline double DynamicPitchFireThreshold(const AimerConfig& cfg,
                                        const Eigen::Vector3d& target_xyz)
{
  const double horizontal_distance =
      std::max(MIN_HORIZONTAL_DISTANCE_M, HorizontalDistance(target_xyz));
  const double pitch_half = std::atan2(0.5 * ARMOR_HEIGHT_M, horizontal_distance);
  const double spread_pitch = std::atan2(FIRE_BULLET_SPREAD_M, horizontal_distance);
  return std::clamp(pitch_half - spread_pitch, cfg.min_fire_threshold,
                    cfg.max_fire_threshold);
}

/**
 * @brief 解算低抛弹道 pitch 和飞行时间。
 * @param bullet_speed 弹速，单位 m/s。
 * @param horizontal_distance tracker x-y 平面距离，单位 m。
 * @param target_height tracker z 方向目标高度，单位 m。
 * @return 弹道解或无解标志。
 */
inline TrajectorySolution SolveTrajectoryPitch(double bullet_speed,
                                               double horizontal_distance,
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
}  // namespace AimerDetail
