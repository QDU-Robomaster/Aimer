#pragma once

/**
 * @file AimerHeatFire.hpp
 * @brief 按热量分配的开火判定：本地热量估计、单发命中概率和随热量升高的开火门槛。
 *        Heat-aware fire decision: local barrel heat estimate, single-shot hit
 *        probability and a fire threshold that rises with heat.
 *
 * 出弹数受热量限制时，可开火的时机多于热量允许的发数。这里估计每个时机的命中概率，
 * 只在概率高于门槛时开火；门槛随热量升高，热量低且长时间未开火时逐步放宽。
 * When the number of shots is limited by heat, there are more fire opportunities than
 * the heat allows. The hit probability of each opportunity is estimated and the shot is
 * fired only above a threshold; the threshold rises with heat and is relaxed step by
 * step when the heat is low and no shot was fired for a while.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace AimerDetail
{
/// 记录的最近计入热量的出弹时刻个数
/// Number of recent counted shot times that are kept
inline constexpr std::size_t HEAT_RECENT_SHOTS = 16;

/**
 * @brief 本地热量估计状态。
 *        Local barrel heat estimate.
 */
struct HeatFireState
{
  /// 最近计入热量的出弹时刻，环形缓冲，单位 us
  /// Times of the recent counted shots, ring buffer, in us
  std::array<uint64_t, HEAT_RECENT_SHOTS> recent_shots_us{};
  /// 环形缓冲中已写入的出弹个数
  /// Number of shots written to the ring buffer
  std::size_t recent_shot_count{0};
  /// 是否已初始化
  /// Whether the state is initialised
  bool valid{false};
  /// 估计热量
  /// Estimated heat
  double heat{0.0};
  /// 估计热量对应的时刻，单位 us
  /// Time of the heat estimate, in us
  uint64_t heat_time_us{0};
  /// 上一发计入热量的时刻，单位 us
  /// Time of the last counted shot, in us
  uint64_t last_shot_us{0};
  /// 放宽门槛的计时起点：上一发计入热量的时刻或开始跟踪当前目标的时刻，取较晚者，单位 us
  /// Start of the relaxation timer: the later of the last counted shot and the start of
  /// tracking the current target, in us
  uint64_t relax_ref_us{0};
  /// 是否已有计入热量的出弹
  /// Whether a shot has been counted
  bool has_shot{false};
};

/**
 * @brief 按冷却值把热量估计推进到当前时刻。
 *        Advance the heat estimate to the current time with the cooling value.
 * @param state 热量估计状态 / Heat estimate state.
 * @param now_us 当前时刻，单位 us / Current time, in us.
 * @param cooling_per_s 每秒冷却值 / Cooling per second.
 */
inline void CoolHeat(HeatFireState& state, uint64_t now_us, double cooling_per_s)
{
  if (!state.valid)
  {
    state = {};
    state.valid = true;
    state.heat_time_us = now_us;
    state.relax_ref_us = now_us;
    return;
  }
  if (now_us > state.heat_time_us)
  {
    const double dt_s = static_cast<double>(now_us - state.heat_time_us) * 1e-6;
    state.heat = std::max(0.0, state.heat - std::max(0.0, cooling_per_s) * dt_s);
    state.heat_time_us = now_us;
  }
}

/**
 * @brief 把一次开火请求计入热量；与上一发间隔小于发射机构最小间隔时不计入。
 *        Count a fire request as a shot; requests closer than the minimum launcher
 *        interval to the previous shot are not counted.
 * @return 计入时返回 true / True when the shot was counted.
 */
inline bool CountShot(HeatFireState& state, uint64_t now_us, double shot_heat,
                      double min_interval_s)
{
  const auto min_interval_us =
      static_cast<uint64_t>(std::llround(std::max(0.0, min_interval_s) * 1e6));
  if (state.has_shot && now_us < state.last_shot_us + min_interval_us)
  {
    return false;
  }
  state.heat += std::max(0.0, shot_heat);
  state.last_shot_us = now_us;
  state.relax_ref_us = std::max(state.relax_ref_us, now_us);
  state.has_shot = true;
  state.recent_shots_us[state.recent_shot_count % HEAT_RECENT_SHOTS] = now_us;
  ++state.recent_shot_count;
  return true;
}

/**
 * @brief 统计晚于给定时刻计入热量的出弹数，最多统计最近 HEAT_RECENT_SHOTS 发。
 *        Count the counted shots later than the given time, at most the last
 *        HEAT_RECENT_SHOTS shots.
 * @param state 热量估计状态 / Heat estimate state.
 * @param since_us 起始时刻，单位 us，不含该时刻 / Start time in us, exclusive.
 * @return 出弹数 / Number of shots.
 */
inline std::size_t CountedShotsSince(const HeatFireState& state, uint64_t since_us)
{
  const std::size_t kept = std::min(state.recent_shot_count, HEAT_RECENT_SHOTS);
  std::size_t count = 0;
  for (std::size_t i = 0; i < kept; ++i)
  {
    if (state.recent_shots_us[i] > since_us)
    {
      ++count;
    }
  }
  return count;
}

/**
 * @brief 用裁判系统实测热量修正本地热量估计。
 *        Correct the local heat estimate with the heat measured by the referee system.
 *
 * 实测热量是本地估计的下限；实测值可能尚未计入最近几发，因此上限是实测值加上这些出弹的
 * 热量。本地估计落在区间内时保持不变，超出时取最近的边界。
 * The measured heat is a lower bound of the local estimate; the measurement may not yet
 * include the latest shots, so the upper bound is the measured heat plus the heat of
 * those shots. A local estimate inside the interval is kept; outside it, the nearest bound
 * is taken.
 *
 * @param state 热量估计状态，已推进到当前时刻 / Heat estimate state, advanced to the
 *        current time.
 * @param measured_heat 推进到当前时刻的实测热量 / Measured heat advanced to the current
 *        time.
 * @param unseen_heat 实测值可能尚未计入的出弹热量 / Heat of the shots the measurement may
 *        not include yet.
 */
inline void FuseMeasuredHeat(HeatFireState& state, double measured_heat, double unseen_heat)
{
  if (!state.valid || !std::isfinite(measured_heat))
  {
    return;
  }
  const double lower = std::max(0.0, measured_heat);
  const double upper = lower + std::max(0.0, unseen_heat);
  state.heat = std::clamp(state.heat, lower, upper);
}

/**
 * @brief 开始跟踪一个目标时重新开始放宽计时，使新目标先按正常门槛挑选时机。
 *        Restart the relaxation timer when a target starts to be tracked, so that a new
 *        target is first judged with the normal threshold.
 */
inline void StartEngagement(HeatFireState& state, uint64_t now_us)
{
  if (state.valid)
  {
    state.relax_ref_us = std::max(state.relax_ref_us, now_us);
  }
}

/**
 * @brief 横向脱靶服从 N(bias, sigma) 时落在 ±half_width 内的概率。
 *        Probability that a lateral miss distributed as N(bias, sigma) falls inside
 *        ±half_width.
 */
inline double HitProbability(double half_width_m, double bias_m, double sigma_m)
{
  if (!(half_width_m > 0.0))
  {
    return 0.0;
  }
  const double sigma = std::max(sigma_m, 1e-6);
  auto phi = [](double x) { return 0.5 * std::erfc(-x / std::sqrt(2.0)); };
  return phi((half_width_m - bias_m) / sigma) - phi((-half_width_m - bias_m) / sigma);
}

/**
 * @brief 出膛时刻横向偏差的均值：请求时刻云台误差留到出膛的部分，加上随指令角速度
 *        增长的偏差。
 *        Mean lateral offset at muzzle exit: the part of the request-time gimbal error
 *        that remains at the exit, plus an offset that grows with the command yaw rate.
 * @param distance_m 目标水平距离，单位 m / Horizontal target distance, in m.
 * @param gimbal_error_rad 请求时刻云台 yaw 减指令 yaw，单位 rad / Gimbal yaw minus
 *        command yaw at the request, in rad.
 * @param error_gain 出膛时保留的云台误差比例 / Share of the gimbal error remaining at
 *        the exit.
 * @param command_yaw_rate 指令 yaw 角速度，单位 rad/s / Command yaw rate, in rad/s.
 * @param rate_bias_s 指令角速度对应的偏差时间，单位 s / Offset time per command yaw
 *        rate, in s.
 * @return 横向偏差绝对值，单位 m / Absolute lateral offset, in m.
 */
inline double ExitLateralBias(double distance_m, double gimbal_error_rad, double error_gain,
                              double command_yaw_rate, double rate_bias_s)
{
  return distance_m * std::abs(error_gain * gimbal_error_rad + rate_bias_s * command_yaw_rate);
}

/**
 * @brief 出膛时刻横向偏差的标准差：基础项、装甲相位项和随指令角速度增长的项。
 *        Standard deviation of the lateral offset at muzzle exit: a base term, the plate
 *        phase term and a term that grows with the command yaw rate.
 * @return 标准差，单位 m / Standard deviation, in m.
 */
inline double ExitLateralSigma(double base_sigma_m, double phase_sigma_m, double distance_m,
                               double command_yaw_rate, double rate_spread_s)
{
  const double rate_sigma = distance_m * rate_spread_s * command_yaw_rate;
  return std::sqrt(base_sigma_m * base_sigma_m + phase_sigma_m * phase_sigma_m +
                   rate_sigma * rate_sigma);
}

/**
 * @brief 开火所需的命中概率。
 *        Hit probability required to fire.
 *
 * 门槛从 p_low（热量为 0）线性升到 p_high（热量达到上限）。热量低于上限一半且距上一发
 * 超过 relax_s 时，门槛在接下来的 relax_s 内线性降到 p_floor，避免可达命中概率普遍偏低
 * 时（远距离、斜对的静止装甲、爆发优先发射机构）一直攒着热量不打。
 * The threshold rises linearly from p_low (zero heat) to p_high (heat at the limit).
 * When the heat is below half the limit and the last shot is older than relax_s, the
 * threshold falls linearly to p_floor over the next relax_s, so that heat is not hoarded
 * when every opportunity has a low hit probability (long range, a still oblique plate,
 * burst-priority launchers).
 */
inline double HeatFireThreshold(double p_low, double p_high, double p_floor, double relax_s,
                                double heat_fraction, double since_shot_s)
{
  const double fraction = std::clamp(heat_fraction, 0.0, 1.0);
  double threshold = p_low + (p_high - p_low) * fraction;
  if (fraction < 0.5 && relax_s > 0.0 && since_shot_s > relax_s)
  {
    const double k = std::min(1.0, (since_shot_s - relax_s) / relax_s);
    threshold -= (threshold - std::min(threshold, p_floor)) * k;
  }
  return threshold;
}
}  // namespace AimerDetail
