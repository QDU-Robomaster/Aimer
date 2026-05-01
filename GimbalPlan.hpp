#pragma once

#ifndef QDU_GIMBAL_PLAN_HPP
#define QDU_GIMBAL_PLAN_HPP

#include <cstdint>

struct GimbalPlan
{
  uint64_t image_timestamp_us{0};
  bool control{false};
  bool fire{false};

  float target_yaw{0.0f};
  float target_pitch{0.0f};

  float yaw{0.0f};
  float yaw_vel{0.0f};
  float yaw_acc{0.0f};

  float pitch{0.0f};
  float pitch_vel{0.0f};
  float pitch_acc{0.0f};
};

#endif  // QDU_GIMBAL_PLAN_HPP
