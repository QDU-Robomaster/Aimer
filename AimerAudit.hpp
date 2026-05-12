#pragma once

/**
 * @file AimerAudit.hpp
 * @brief Aimer 决策的 TSV 审计写入和字符串转换工具。
 */

#include <iomanip>
#include <string_view>

#include "logger.hpp"

namespace AimerDetail
{
/**
 * @brief 将 Aimer 策略转换为审计字符串。
 * @param strategy 策略枚举值。
 * @return TSV 输出使用的稳定字符串。
 */
inline const char* ToString(Aimer::Strategy strategy)
{
  switch (strategy)
  {
    case Aimer::Strategy::LOST:
      return "lost";
    case Aimer::Strategy::LOW_SPEED:
      return "low_speed";
    case Aimer::Strategy::MEDIUM_SPIN:
      return "medium_spin";
    case Aimer::Strategy::OUTPOST:
      return "outpost";
  }
  return "unknown";
}

/**
 * @brief 将选择原因转换为审计字符串。
 * @param reason 选择原因枚举值。
 * @return TSV 输出使用的稳定字符串。
 */
inline const char* ToString(Aimer::SelectReason reason)
{
  switch (reason)
  {
    case Aimer::SelectReason::NONE:
      return "none";
    case Aimer::SelectReason::NEAREST_FRONT:
      return "nearest_front";
  }
  return "unknown";
}

/**
 * @brief 将切换原因转换为审计字符串。
 * @param reason 切换原因枚举值。
 * @return TSV 输出使用的稳定字符串。
 */
inline const char* ToString(Aimer::SwitchReason reason)
{
  switch (reason)
  {
    case Aimer::SwitchReason::NONE:
      return "none";
    case Aimer::SwitchReason::NEW_TARGET:
      return "new_target";
    case Aimer::SwitchReason::NEAREST_CHANGED:
      return "nearest_changed";
  }
  return "unknown";
}

/**
 * @brief 将开火原因转换为审计字符串。
 * @param reason 开火原因枚举值。
 * @return TSV 输出使用的稳定字符串。
 */
inline const char* ToString(Aimer::FireReason reason)
{
  switch (reason)
  {
    case Aimer::FireReason::DISABLED:
      return "disabled";
    case Aimer::FireReason::OK:
      return "ok";
    case Aimer::FireReason::NO_TRACK:
      return "no_track";
    case Aimer::FireReason::NO_GIMBAL:
      return "no_gimbal";
    case Aimer::FireReason::COMMAND_UNSTABLE:
      return "command_unstable";
    case Aimer::FireReason::GIMBAL_NOT_ALIGNED:
      return "gimbal_not_aligned";
    case Aimer::FireReason::NOT_SHOOTABLE:
      return "not_shootable";
    case Aimer::FireReason::BALLISTIC_UNSOLVABLE:
      return "ballistic_unsolvable";
  }
  return "unknown";
}

/**
 * @brief 将装甲板数字枚举转换为配置中的显示名。
 * @param number 装甲板数字枚举值。
 * @return 显示名；越界值返回 "invalid"。
 */
[[maybe_unused]] inline std::string_view ArmorNumberToString(ArmorNumber number)
{
  const std::size_t index = static_cast<std::size_t>(number);
  if (index >= ARMOR_NUMBER_NAMES.size())
  {
    return std::string_view{"invalid"};
  }
  return ARMOR_NUMBER_NAMES[index];
}
}  // namespace AimerDetail

/**
 * @brief 将当前帧决策写入可选 TSV 审计文件。
 */
inline void Aimer::WriteDecisionAudit()
{
  if (decision_audit_.path.empty())
  {
    return;
  }
  if (!decision_audit_.file.is_open())
  {
    decision_audit_.file.open(decision_audit_.path, std::ios::out | std::ios::trunc);
    if (!decision_audit_.file)
    {
      if (!decision_audit_.open_failed)
      {
        XR_LOG_ERROR("Aimer failed to open decision audit: %s",
                     decision_audit_.path.c_str());
        decision_audit_.open_failed = true;
      }
      return;
    }
    decision_audit_.file << std::setprecision(9);
    decision_audit_.file
        << "frame_id\timage_timestamp_us\taimer_receive_time_us\tpredict_time_us\t"
        << "expected_hit_time_us\ttarget_tracking\tvalid\tconverged\t"
        << "candidate_count\tselected_armor_index\ttarget_id\t"
        << "strategy\tselected_reason\tswitch_reason\tfire_reason\t"
        << "fixed_delay_s\tfire_delay_s\tfly_time_s\ttotal_hit_delay_s\t"
        << "selected_x\tselected_y\tselected_z\tselected_yaw\tselected_view_angle\t"
        << "selected_front_facing\tshootable\tcommand_yaw\tcommand_pitch\t"
        << "target_yaw\ttarget_pitch\tplanned_yaw\tplanned_pitch\t"
        << "planned_yaw_vel\tplanned_pitch_vel\tplanned_yaw_acc\tplanned_pitch_acc\t"
        << "mpc_used\tfire_allowed\tfire_thres_yaw\tfire_thres_pitch\t"
        << "command_error_yaw\tcommand_error_pitch\tactual_gimbal_error_yaw\t"
        << "actual_gimbal_error_pitch\n";
  }

  decision_audit_.file
      << decision_msg_.frame_id << '\t' << decision_msg_.image_timestamp_us << '\t'
      << decision_msg_.aimer_receive_time_us << '\t' << decision_msg_.predict_time_us
      << '\t' << decision_msg_.expected_hit_time_us << '\t'
      << (decision_msg_.target_tracking ? 1 : 0) << '\t'
      << (decision_msg_.valid ? 1 : 0) << '\t'
      << (decision_msg_.converged ? 1 : 0) << '\t'
      << static_cast<int>(decision_msg_.candidate_count) << '\t'
      << static_cast<int>(decision_msg_.selected_armor_index) << '\t'
      << static_cast<int>(decision_msg_.target_id) << '\t'
      << AimerDetail::ToString(decision_msg_.strategy) << '\t'
      << AimerDetail::ToString(decision_msg_.selected_reason) << '\t'
      << AimerDetail::ToString(decision_msg_.switch_reason) << '\t'
      << AimerDetail::ToString(decision_msg_.fire_reason) << '\t'
      << decision_msg_.fixed_delay_s << '\t' << decision_msg_.fire_delay_s << '\t'
      << decision_msg_.fly_time_s << '\t' << decision_msg_.total_hit_delay_s
      << '\t' << decision_msg_.selected_x << '\t' << decision_msg_.selected_y
      << '\t' << decision_msg_.selected_z << '\t' << decision_msg_.selected_yaw
      << '\t' << decision_msg_.selected_view_angle << '\t'
      << (decision_msg_.selected_front_facing ? 1 : 0) << '\t'
      << (decision_msg_.shootable ? 1 : 0) << '\t'
      << decision_msg_.command_yaw << '\t' << decision_msg_.command_pitch << '\t'
      << decision_msg_.target_yaw << '\t' << decision_msg_.target_pitch << '\t'
      << decision_msg_.planned_yaw << '\t' << decision_msg_.planned_pitch << '\t'
      << decision_msg_.planned_yaw_vel << '\t' << decision_msg_.planned_pitch_vel
      << '\t' << decision_msg_.planned_yaw_acc << '\t'
      << decision_msg_.planned_pitch_acc << '\t'
      << (decision_msg_.mpc_used ? 1 : 0) << '\t'
      << (decision_msg_.fire_allowed ? 1 : 0) << '\t'
      << decision_msg_.fire_thres_yaw << '\t'
      << decision_msg_.fire_thres_pitch << '\t'
      << decision_msg_.command_error_yaw << '\t'
      << decision_msg_.command_error_pitch << '\t'
      << decision_msg_.actual_gimbal_error_yaw << '\t'
      << decision_msg_.actual_gimbal_error_pitch << '\n';
  decision_audit_.file.flush();
}

/**
 * @brief 将已触发开火的 shot 事件写入可选 TSV 审计文件。
 * @param shot 需要持久化的 shot 事件载荷。
 */
inline void Aimer::WriteShotAudit(const Aimer::AimerShotEvent& shot)
{
  if (shot_audit_.path.empty())
  {
    return;
  }
  if (!shot_audit_.file.is_open())
  {
    shot_audit_.file.open(shot_audit_.path, std::ios::out | std::ios::trunc);
    if (!shot_audit_.file)
    {
      if (!shot_audit_.open_failed)
      {
        XR_LOG_ERROR("Aimer failed to open shot audit: %s",
                     shot_audit_.path.c_str());
        shot_audit_.open_failed = true;
      }
      return;
    }
    shot_audit_.file << std::setprecision(9);
    shot_audit_.file
        << "shot_id\tframe_id\timage_timestamp_us\tcommand_time_us\t"
        << "expected_hit_time_us\tselected_armor_index\ttarget_id\tcommand_yaw\t"
        << "command_pitch\tactual_gimbal_yaw\tactual_gimbal_pitch\tbullet_speed\t"
        << "fire_delay_s\tfly_time_est_s\tfire_reason\n";
  }
  shot_audit_.file << shot.shot_id << '\t' << shot.frame_id << '\t'
                   << shot.image_timestamp_us << '\t' << shot.command_time_us
                   << '\t' << shot.expected_hit_time_us << '\t'
                   << static_cast<int>(shot.selected_armor_index) << '\t'
                   << static_cast<int>(shot.target_id) << '\t'
                   << shot.command_yaw << '\t' << shot.command_pitch << '\t'
                   << shot.actual_gimbal_yaw << '\t' << shot.actual_gimbal_pitch
                   << '\t' << shot.bullet_speed << '\t' << shot.fire_delay_s
                   << '\t' << shot.fly_time_est_s << '\t'
                   << AimerDetail::ToString(shot.fire_reason) << '\n';
  shot_audit_.file.flush();
}
