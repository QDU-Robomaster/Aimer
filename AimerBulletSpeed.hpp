#pragma once

/**
 * @file AimerBulletSpeed.hpp
 * @brief 实测弹速的鲁棒平滑：滑动窗口中位数剔除离群值后取均值。
 *        Robust smoothing of the measured bullet speed: the mean of a sliding window
 *        after the outliers around the window median are removed.
 *
 * 裁判系统每发弹丸给出一次初速度。单发值有测速噪声，偶尔出现明显偏离的读数。这里保存
 * 最近若干发的有效读数，以窗口中位数为中心按中位数绝对偏差（不小于给定下限）划定门限，
 * 门限内读数的均值作为弹速估计。弹速整体变化时，新读数在窗口中占多数后中位数随之移动，
 * 估计跟上新的弹速。
 * The referee system reports the initial speed of every projectile. A single reading has
 * measurement noise and occasionally deviates clearly. The valid readings of the last shots
 * are kept; a gate around the window median is set from the median absolute deviation (not
 * below a given floor), and the mean of the readings inside the gate is the bullet-speed
 * estimate. When the speed changes as a whole, the median moves once the new readings form
 * the majority of the window, and the estimate follows the new speed.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace AimerDetail
{
/**
 * @brief 弹速平滑参数。
 *        Bullet-speed smoothing parameters.
 */
struct BulletSpeedFilterParam
{
  /// 窗口长度，限制在 1 到 BulletSpeedFilter::MAX_WINDOW
  /// Window length, limited to 1 to BulletSpeedFilter::MAX_WINDOW
  int window{9};
  /// 给出估计所需的最少有效读数
  /// Minimum number of valid readings before an estimate is given
  int min_samples{3};
  /// 有效读数下限，单位 m/s
  /// Lower bound of a valid reading, in m/s
  double min_speed{14.0};
  /// 有效读数上限，单位 m/s
  /// Upper bound of a valid reading, in m/s
  double max_speed{35.0};
  /// 离群门限的下限，单位 m/s
  /// Floor of the outlier gate, in m/s
  double outlier_floor{0.5};
  /// 离群门限相对标准化中位数绝对偏差的倍数
  /// Outlier gate as a multiple of the normalized median absolute deviation
  double outlier_mad_scale{3.0};
};

/**
 * @brief 实测弹速的滑动窗口鲁棒平滑。
 *        Sliding-window robust smoothing of the measured bullet speed.
 */
class BulletSpeedFilter
{
 public:
  /// 窗口长度上限
  /// Maximum window length
  static constexpr std::size_t MAX_WINDOW = 32;

  explicit BulletSpeedFilter(const BulletSpeedFilterParam& param = {})
      : param_(param),
        window_(static_cast<std::size_t>(
            std::clamp(param.window, 1, static_cast<int>(MAX_WINDOW)))),
        min_samples_(static_cast<std::size_t>(
            std::clamp(param.min_samples, 1, static_cast<int>(window_))))
  {
  }

  /**
   * @brief 加入一发的实测弹速。
   *        Add the measured speed of one shot.
   * @param speed 弹速，单位 m/s / Speed, in m/s.
   * @return 读数有效并已加入窗口时为 true / True when the reading is valid and added.
   */
  bool AddSample(double speed)
  {
    if (!std::isfinite(speed) || speed < param_.min_speed || speed > param_.max_speed)
    {
      return false;
    }
    samples_[next_] = speed;
    next_ = (next_ + 1) % window_;
    count_ = std::min(count_ + 1, window_);
    return true;
  }

  /**
   * @brief 当前窗口中的有效读数个数。
   *        Number of valid readings in the window.
   */
  [[nodiscard]] std::size_t Count() const { return count_; }

  /**
   * @brief 给出弹速估计。
   *        Give the bullet-speed estimate.
   * @param estimate 输出的弹速估计，单位 m/s / Output estimate, in m/s.
   * @return 有效读数足够时为 true / True when there are enough valid readings.
   */
  bool Estimate(double& estimate) const
  {
    if (count_ < min_samples_)
    {
      return false;
    }
    std::array<double, MAX_WINDOW> sorted{};
    std::copy_n(samples_.begin(), count_, sorted.begin());
    const double median = Median(sorted, count_);

    std::array<double, MAX_WINDOW> deviation{};
    for (std::size_t i = 0; i < count_; ++i)
    {
      deviation[i] = std::abs(samples_[i] - median);
    }
    constexpr double MAD_TO_SIGMA = 1.4826;
    const double gate = std::max(param_.outlier_floor,
                                 param_.outlier_mad_scale * MAD_TO_SIGMA *
                                     Median(deviation, count_));

    double sum = 0.0;
    std::size_t used = 0;
    for (std::size_t i = 0; i < count_; ++i)
    {
      if (std::abs(samples_[i] - median) <= gate)
      {
        sum += samples_[i];
        ++used;
      }
    }
    estimate = used > 0 ? sum / static_cast<double>(used) : median;
    return true;
  }

 private:
  /// 前 count 个元素的中位数，会重排这些元素
  /// Median of the first count elements; the elements are reordered
  static double Median(std::array<double, MAX_WINDOW>& values, std::size_t count)
  {
    std::sort(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(count));
    const std::size_t mid = count / 2;
    return count % 2 == 1 ? values[mid] : 0.5 * (values[mid - 1] + values[mid]);
  }

  BulletSpeedFilterParam param_;
  std::size_t window_;
  std::size_t min_samples_;
  std::array<double, MAX_WINDOW> samples_{};
  std::size_t next_{0};
  std::size_t count_{0};
};
}  // namespace AimerDetail
