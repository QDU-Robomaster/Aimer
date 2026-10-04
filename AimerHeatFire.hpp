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
#include <cmath>
#include <cstdint>

namespace AimerDetail
{
/**
 * @brief 本地热量估计状态。
 *        Local barrel heat estimate.
 */
struct HeatFireState
{
  /// 是否已初始化
  /// Whether the state is initialised
  bool valid{false};
  /// 估计热量
  /// Estimated heat
  double heat{0.0};
  /// 估计热量对应的时刻，单位 us
  /// Time of the heat estimate, in us
  uint64_t heat_time_us{0};
  /// 上一发计入热量的时刻；初始化时为初始化时刻，单位 us
  /// Time of the last counted shot; the initialisation time before the first shot, in us
  uint64_t last_shot_us{0};
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
    state.last_shot_us = now_us;
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
  state.has_shot = true;
  return true;
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
