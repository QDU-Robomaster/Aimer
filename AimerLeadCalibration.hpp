#pragma once

/**
 * @file AimerLeadCalibration.hpp
 * @brief 指向超前量的在线标定：用之后的帧检查云台指向，按批修正预测延迟，收敛后固定。
 *        Online calibration of the pointing lead: the gimbal pointing is checked against
 *        later frames, the prediction delay is corrected in batches and frozen once
 *        calibrated.
 *
 * 每帧记下云台实测 yaw（与图像同步的 IMU）和当时生效指令的 yaw 角速度。子弹若在该帧
 * 图像时刻出膛，会在一个飞行时间后到达；到达时刻之后的第一帧直接观测到装甲板，外推几
 * 毫秒即得到达时刻的装甲板方位。两者之差对指令角速度做回归，斜率就是指向超前的时间：
 * 为正表示云台沿目标运动方向超前。该检查只用 Aimer 每帧已有的数据，不需要开火或真值。
 * For every frame the measured gimbal yaw (IMU synced to the image) and the yaw rate of the
 * command in force are kept. A projectile leaving at that image time arrives one flight
 * time later; the first frame imaged after the arrival observes the plate directly, and a
 * few milliseconds of propagation give the plate bearing at the arrival. The difference is
 * regressed on the command yaw rate; the slope is the pointing lead time, positive when the
 * gimbal is ahead along the target motion. The check uses only data Aimer has in every
 * frame and needs neither shots nor ground truth.
 */

#include <algorithm>
#include <cmath>

namespace AimerDetail
{
/**
 * @brief 在线标定的参数。
 *        Parameters of the online calibration.
 */
struct LeadCalibrationParam
{
  /// 修正批数，之后固定修正量 / Number of correcting batches before the correction is frozen
  int calibration_batches{5};
  /// 修正量绝对值上限，单位 s / Limit of the absolute correction, in s
  double max_adjust_s{0.05};
  /// 固定后触发重新标定的超前时间，单位 s / Lead time that triggers a recalibration once
  /// frozen, in s
  double monitor_threshold_s{0.003};
  /// 触发重新标定所需的同向连续批数 / Consecutive same-sign batches that trigger a
  /// recalibration
  int monitor_batches{3};
};

/**
 * @brief 按批回归指向超前时间并修正预测延迟。
 *        Batch regression of the pointing lead time and correction of the prediction
 *        delay.
 *
 * 前两批每批 250 帧、修正增益 3，之后每批 400 帧、增益 1.5；指令角速度方差不足的批次
 * 不使用。达到修正批数后，修正量固定为最后三批修正量的平均，之后只监视：连续多批超前
 * 时间同号且超过门槛时重新标定。
 * The first two batches have 250 frames and a correction gain of 3, later batches 400
 * frames and a gain of 1.5; batches with too little command-rate variance are not used.
 * After the correcting batches the correction is frozen at the mean of the last three
 * corrections; afterwards the lead is only monitored, and a recalibration starts when
 * several consecutive batches exceed the threshold with the same sign.
 */
class LeadCalibrator
{
 public:
  /// 一批结束时的结果 / Outcome of a closed batch
  enum class Event
  {
    NONE,        ///< 批未满 / Batch not full
    SKIPPED,     ///< 指令角速度方差不足 / Too little command-rate variance
    CORRECTED,   ///< 已修正 / Correction applied
    FROZEN,      ///< 已修正并固定 / Correction applied and frozen
    MONITORED,   ///< 固定后的监视批 / Monitoring batch after freezing
    RESTARTED,   ///< 监视发现偏离，重新标定 / Deviation found, recalibration started
  };

  /// 一批的结果 / Result of a batch
  struct Result
  {
    Event event{Event::NONE};
    double lead_s{0.0};    ///< 本批回归的超前时间 / Lead time of this batch
    double adjust_s{0.0};  ///< 当前修正量 / Current correction
    double samples{0.0};   ///< 本批样本数 / Samples in this batch
  };

  explicit LeadCalibrator(LeadCalibrationParam param = {}) : param_(param) {}

  /**
   * @brief 加入一个样本。
   *        Add one sample.
   * @param command_yaw_rate 指令 yaw 角速度，单位 rad/s / Command yaw rate, in rad/s.
   * @param residual_rad 云台 yaw 减到达时刻装甲板方位，单位 rad / Gimbal yaw minus the
   *        plate bearing at the arrival, in rad.
   */
  void AddSample(double command_yaw_rate, double residual_rad)
  {
    n_ += 1.0;
    sw_ += command_yaw_rate;
    se_ += residual_rad;
    sww_ += command_yaw_rate * command_yaw_rate;
    swe_ += command_yaw_rate * residual_rad;
  }

  /**
   * @brief 当前批是否已满。
   *        Whether the current batch is full.
   */
  [[nodiscard]] bool BatchReady() const { return n_ >= BatchFrames(); }

  /**
   * @brief 结束当前批：回归超前时间，按阶段修正、固定或监视。
   *        Close the current batch: regress the lead time, then correct, freeze or
   *        monitor depending on the stage.
   */
  Result CloseBatch()
  {
    Result result{};
    result.samples = n_;
    const double mean_w = sw_ / std::max(n_, 1.0);
    const double var_w = sww_ / std::max(n_, 1.0) - mean_w * mean_w;
    const double cov_we = swe_ / std::max(n_, 1.0) - mean_w * (se_ / std::max(n_, 1.0));
    ResetBatch();
    result.adjust_s = adjust_s_;
    if (!(var_w >= MIN_RATE_VARIANCE))
    {
      result.event = Event::SKIPPED;
      return result;
    }
    const double lead_s = cov_we / var_w;
    result.lead_s = lead_s;

    if (!frozen_)
    {
      const double gain = batch_ < 2 ? 3.0 : 1.5;
      adjust_s_ = std::clamp(adjust_s_ - gain * lead_s, -param_.max_adjust_s, param_.max_adjust_s);
      recent_[batch_ % 3] = adjust_s_;
      ++batch_;
      result.event = Event::CORRECTED;
      if (batch_ >= std::max(1, param_.calibration_batches))
      {
        const int count = std::min(batch_, 3);
        double sum = 0.0;
        for (int i = 0; i < count; ++i)
        {
          sum += recent_[i];
        }
        adjust_s_ = sum / count;
        frozen_ = true;
        drift_count_ = 0;
        result.event = Event::FROZEN;
      }
      result.adjust_s = adjust_s_;
      return result;
    }

    const int sign = lead_s > param_.monitor_threshold_s    ? 1
                     : lead_s < -param_.monitor_threshold_s ? -1
                                                            : 0;
    drift_count_ = (sign != 0 && sign == drift_sign_) ? drift_count_ + 1 : (sign != 0 ? 1 : 0);
    drift_sign_ = sign;
    result.event = Event::MONITORED;
    if (drift_count_ >= std::max(1, param_.monitor_batches))
    {
      frozen_ = false;
      batch_ = 0;
      drift_count_ = 0;
      drift_sign_ = 0;
      result.event = Event::RESTARTED;
    }
    return result;
  }

  /**
   * @brief 清空当前批的累加量，用于修正生效前的样本作废。
   *        Clear the sums of the current batch, e.g. to drop samples taken before a
   *        correction took effect.
   */
  void ResetBatch() { n_ = sw_ = se_ = sww_ = swe_ = 0.0; }

  /// 当前修正量，加到预测延迟上，单位 s / Current correction added to the prediction
  /// delay, in s
  [[nodiscard]] double Adjust() const { return adjust_s_; }
  /// 修正量是否已固定 / Whether the correction is frozen
  [[nodiscard]] bool Frozen() const { return frozen_; }

 private:
  static constexpr double MIN_RATE_VARIANCE = 0.15 * 0.15;

  [[nodiscard]] double BatchFrames() const { return (!frozen_ && batch_ < 2) ? 250.0 : 400.0; }

  LeadCalibrationParam param_{};
  double n_{0.0};
  double sw_{0.0};
  double se_{0.0};
  double sww_{0.0};
  double swe_{0.0};
  double adjust_s_{0.0};
  double recent_[3]{0.0, 0.0, 0.0};
  int batch_{0};
  bool frozen_{false};
  int drift_sign_{0};
  int drift_count_{0};
};
}  // namespace AimerDetail
