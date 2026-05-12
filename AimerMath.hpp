#pragma once

/**
 * @file AimerMath.hpp
 * @brief Ballistic and angle math helpers for the aimer implementation.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

#include <Eigen/Dense>

namespace AimerDetail
{
inline constexpr double PI = 3.14159265358979323846;
inline constexpr double DEG2RAD = PI / 180.0;
inline constexpr double GRAVITY = 9.7833;
inline constexpr double MIN_HORIZONTAL_DISTANCE_M = 1e-4;

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

inline uint64_t SecondsToMicros(double seconds)
{
  if (!std::isfinite(seconds) || seconds <= 0.0)
  {
    return 0;
  }
  return static_cast<uint64_t>(std::llround(seconds * 1.0e6));
}

template <typename Derived>
inline double HorizontalDistance(const Eigen::MatrixBase<Derived>& point)
{
  return std::hypot(point.x(), point.z());
}

template <typename Derived>
inline double BearingYaw(const Eigen::MatrixBase<Derived>& point)
{
  return std::atan2(point.z(), point.x());
}

template <typename Derived>
inline double BallisticHeight(const Eigen::MatrixBase<Derived>& point)
{
  return point.y();
}

struct TrajectorySolution
{
  bool unsolvable{false};
  double fly_time{0.0};
  double pitch{0.0};
};

inline std::pair<double, double> DynamicFireThreshold(
    const Aimer::Config& cfg, const Eigen::Vector3d& target_xyz,
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
