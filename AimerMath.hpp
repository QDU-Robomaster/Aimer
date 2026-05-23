#pragma once

/**
 * @file AimerMath.hpp
 * @brief Aimer 实现使用的角度和弹道数学工具。
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

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
/// RK4 弹道积分步长夹紧范围，避免配置错误造成过慢或数值过粗。
inline constexpr double MIN_BALLISTIC_DT_S = 1e-4;
inline constexpr double MAX_BALLISTIC_DT_S = 0.02;
/// 弹道仿真的最大飞行时间，超过视为不可解。
inline constexpr double MAX_BALLISTIC_FLIGHT_TIME_S = 3.0;
/// RoboMaster 小装甲板规则宽度，单位 m。
inline constexpr double SMALL_ARMOR_WIDTH_M = 0.135;
/// RoboMaster 装甲板规则高度，单位 m。
inline constexpr double ARMOR_HEIGHT_M = 0.056;
/// 自动开火角阈值预留的弹道散布裕量，单位 m。
inline constexpr double FIRE_BULLET_SPREAD_M = 0.015;
/// RoboMaster 装甲板固定安装倾角，单位 deg。
inline constexpr double ARMOR_TILT_DEG = 15.0;
/// TinyMPC ADMM 最大迭代次数。
inline constexpr int MPC_MAX_ITER = 6;

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
 * @brief 弹道仰角解算结果。
 */
struct TrajectorySolution
{
  /// 是否不存在有效弹道解。
  bool unsolvable{false};
  /// 估计弹丸飞行时间，单位 s。
  double fly_time{0.0};
  /// 加配置 roll 轴偏置前的发射仰角，单位 rad。
  double elevation{0.0};
};

struct BallisticSample
{
  bool valid{false};
  double height{0.0};
  double fly_time{0.0};
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
 * @brief 为选中装甲板计算动态 roll 轴开火阈值。
 * @param cfg Aimer 运行时配置。
 * @param target_xyz tracker 输出 B 坐标系下的选中瞄点。
 * @return roll 轴阈值，单位 rad。
 */
inline double DynamicRollFireThreshold(const AimerConfig& cfg,
                                        const Eigen::Vector3d& target_xyz)
{
  const double horizontal_distance =
      std::max(MIN_HORIZONTAL_DISTANCE_M, HorizontalDistance(target_xyz));
  const double roll_half = std::atan2(0.5 * ARMOR_HEIGHT_M, horizontal_distance);
  const double spread_roll = std::atan2(FIRE_BULLET_SPREAD_M, horizontal_distance);
  return std::clamp(roll_half - spread_roll, cfg.min_fire_threshold,
                    cfg.max_fire_threshold);
}

/**
 * @brief 二次阻力弹道状态导数。
 */
inline Eigen::Vector4d BallisticDerivative(const Eigen::Vector4d& state,
                                           double drag_k)
{
  const double vx = state[2];
  const double vz = state[3];
  const double speed = std::hypot(vx, vz);
  Eigen::Vector4d derivative{};
  derivative[0] = vx;
  derivative[1] = vz;
  derivative[2] = -drag_k * speed * vx;
  derivative[3] = -GRAVITY - drag_k * speed * vz;
  return derivative;
}

inline BallisticSample SimulateQuadraticDragTrajectory(
    double bullet_speed, double horizontal_distance, double elevation,
    double drag_k, double integration_dt_s)
{
  BallisticSample sample{};
  if (bullet_speed <= 0.0 || horizontal_distance <= MIN_HORIZONTAL_DISTANCE_M)
  {
    return sample;
  }

  const double dt =
      std::clamp(integration_dt_s, MIN_BALLISTIC_DT_S, MAX_BALLISTIC_DT_S);
  const double k = std::max(0.0, drag_k);
  Eigen::Vector4d state;
  state << 0.0, 0.0, bullet_speed * std::cos(elevation),
      bullet_speed * std::sin(elevation);

  double time = 0.0;
  while (time < MAX_BALLISTIC_FLIGHT_TIME_S)
  {
    const Eigen::Vector4d previous = state;
    const double previous_time = time;
    const Eigen::Vector4d k1 = BallisticDerivative(state, k);
    const Eigen::Vector4d k2 = BallisticDerivative(state + 0.5 * dt * k1, k);
    const Eigen::Vector4d k3 = BallisticDerivative(state + 0.5 * dt * k2, k);
    const Eigen::Vector4d k4 = BallisticDerivative(state + dt * k3, k);
    state += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    time += dt;

    if (!state.allFinite() || state[2] <= 0.0)
    {
      return sample;
    }

    if (state[0] >= horizontal_distance)
    {
      const double dx = state[0] - previous[0];
      const double ratio =
          dx > 1e-9 ? (horizontal_distance - previous[0]) / dx : 0.0;
      const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
      sample.valid = true;
      sample.height =
          previous[1] + clamped_ratio * (state[1] - previous[1]);
      sample.fly_time = previous_time + clamped_ratio * dt;
      return sample;
    }
  }

  return sample;
}

/**
 * @brief 解算无阻力解析弹道仰角和飞行时间。
 * @param bullet_speed 弹速，单位 m/s。
 * @param horizontal_distance tracker x-y 平面距离，单位 m。
 * @param target_height tracker z 方向目标高度，单位 m。
 * @return 弹道解或无解标志。
 */
inline TrajectorySolution SolveTrajectoryElevationNoDrag(
    double bullet_speed, double horizontal_distance, double target_height,
    double min_elevation_deg, double max_elevation_deg)
{
  TrajectorySolution solution;
  if (bullet_speed <= 0.0 || horizontal_distance <= MIN_HORIZONTAL_DISTANCE_M)
  {
    solution.unsolvable = true;
    return solution;
  }

  double elevation_min = min_elevation_deg * DEG2RAD;
  double elevation_max = max_elevation_deg * DEG2RAD;
  if (elevation_min > elevation_max)
  {
    std::swap(elevation_min, elevation_max);
  }

  const double v2 = bullet_speed * bullet_speed;
  const double discriminant =
      v2 * v2 - GRAVITY *
                    (GRAVITY * horizontal_distance * horizontal_distance +
                     2.0 * target_height * v2);
  if (!std::isfinite(discriminant) || discriminant < 0.0)
  {
    solution.unsolvable = true;
    return solution;
  }

  const double sqrt_term = std::sqrt(std::max(0.0, discriminant));
  const double denominator = GRAVITY * horizontal_distance;
  if (!std::isfinite(denominator) || std::abs(denominator) <= 1e-9)
  {
    solution.unsolvable = true;
    return solution;
  }

  const double tan_candidates[2] = {
      (v2 - sqrt_term) / denominator,
      (v2 + sqrt_term) / denominator,
  };

  for (double tan_theta : tan_candidates)
  {
    if (!std::isfinite(tan_theta))
    {
      continue;
    }

    const double elevation = std::atan(tan_theta);
    if (!std::isfinite(elevation) || elevation < elevation_min ||
        elevation > elevation_max)
    {
      continue;
    }

    const double cos_elevation = std::cos(elevation);
    if (!std::isfinite(cos_elevation) || std::abs(cos_elevation) <= 1e-6)
    {
      continue;
    }

    const double fly_time = horizontal_distance / (bullet_speed * cos_elevation);
    if (!std::isfinite(fly_time) || fly_time <= 0.0 ||
        fly_time > MAX_BALLISTIC_FLIGHT_TIME_S)
    {
      continue;
    }

    solution.unsolvable = false;
    solution.elevation = elevation;
    solution.fly_time = fly_time;
    return solution;
  }

  solution.unsolvable = true;
  return solution;
}

/**
 * @brief 解算二次阻力低抛弹道仰角和飞行时间。
 * @param bullet_speed 弹速，单位 m/s。
 * @param horizontal_distance tracker x-y 平面距离，单位 m。
 * @param target_height tracker z 方向目标高度，单位 m。
 * @return 弹道解或无解标志。
 */
inline TrajectorySolution SolveTrajectoryElevation(double bullet_speed,
                                                   double horizontal_distance,
                                                   double target_height,
                                                   double drag_k,
                                                   double integration_dt_s,
                                                   int max_iterations,
                                                   double min_elevation_deg,
                                                   double max_elevation_deg)
{
  TrajectorySolution solution;
  if (bullet_speed <= 0.0 || horizontal_distance <= MIN_HORIZONTAL_DISTANCE_M)
  {
    solution.unsolvable = true;
    return solution;
  }

  double elevation_min = min_elevation_deg * DEG2RAD;
  double elevation_max = max_elevation_deg * DEG2RAD;
  if (elevation_min > elevation_max)
  {
    std::swap(elevation_min, elevation_max);
  }

  auto residual = [&](double elevation, BallisticSample* output = nullptr) {
    const BallisticSample sample = SimulateQuadraticDragTrajectory(
        bullet_speed, horizontal_distance, elevation, drag_k, integration_dt_s);
    if (output != nullptr)
    {
      *output = sample;
    }
    return sample.valid ? sample.height - target_height
                        : std::numeric_limits<double>::quiet_NaN();
  };

  BallisticSample low_sample;
  BallisticSample high_sample;
  double f_low = residual(elevation_min, &low_sample);
  const double f_high = residual(elevation_max, &high_sample);
  if (!std::isfinite(f_low) || !std::isfinite(f_high) ||
      f_low * f_high > 0.0)
  {
    solution.unsolvable = true;
    return solution;
  }

  BallisticSample mid_sample{};
  double elevation_mid = 0.5 * (elevation_min + elevation_max);
  const int iterations = std::clamp(max_iterations, 4, 64);
  for (int i = 0; i < iterations; ++i)
  {
    elevation_mid = 0.5 * (elevation_min + elevation_max);
    const double f_mid = residual(elevation_mid, &mid_sample);
    if (!std::isfinite(f_mid))
    {
      solution.unsolvable = true;
      return solution;
    }
    if (std::abs(f_mid) < 1e-4)
    {
      break;
    }
    if (f_low * f_mid <= 0.0)
    {
      elevation_max = elevation_mid;
    }
    else
    {
      elevation_min = elevation_mid;
      f_low = f_mid;
    }
  }

  elevation_mid = 0.5 * (elevation_min + elevation_max);
  const double f_mid = residual(elevation_mid, &mid_sample);
  if (!std::isfinite(f_mid))
  {
    solution.unsolvable = true;
    return solution;
  }

  solution.unsolvable = false;
  solution.elevation = elevation_mid;
  solution.fly_time = mid_sample.fly_time;
  return solution;
}
}  // namespace AimerDetail
