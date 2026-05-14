#pragma once

/**
 * @file GimbalPlan.hpp
 * @brief Aimer 内部云台计划命令载荷。
 */

#ifndef QDU_GIMBAL_PLAN_HPP
#define QDU_GIMBAL_PLAN_HPP

#include <cstdint>

/**
 * @brief 从 Aimer 规划器采样得到的内部云台前馈计划。
 */
struct GimbalPlan
{
  /// 来源 tracker 帧时间戳，单位 us。
  uint64_t image_timestamp_us{0};
  /// 下级控制器是否应使用该命令。
  bool control{false};
  /// 当前命令点是否允许继续进入最终开火门控。
  bool fire{false};

  /// 规划器输出点的参考 yaw，单位 rad。
  float target_yaw{0.0f};
  /// 规划器输出点的参考 roll 轴命令，单位 rad。
  float target_roll{0.0f};

  /// 规划后的 yaw 命令，单位 rad。
  float yaw{0.0f};
  /// 规划后的 yaw 速度前馈，单位 rad/s。
  float yaw_vel{0.0f};
  /// 规划后的 yaw 加速度前馈，单位 rad/s^2。
  float yaw_acc{0.0f};

  /// 规划后的 roll 轴命令，单位 rad。
  float roll{0.0f};
  /// 规划后的 roll 轴速度前馈，单位 rad/s。
  float roll_vel{0.0f};
  /// 规划后的 roll 轴加速度前馈，单位 rad/s^2。
  float roll_acc{0.0f};
};

#endif  // QDU_GIMBAL_PLAN_HPP
