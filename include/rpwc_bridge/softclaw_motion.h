#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

namespace rpwc_bridge
{

struct SoftClawMotionConfig
{
  double position_tolerance = 0.02;
  double target_stability_sec = 0.2;
  double early_stability_sec = 0.5;
  double stability_delta = 0.01;
  double min_close_progress = 0.02;
  double motion_timeout_sec = 8.0;
};

class SoftClawMotion
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  enum class Result { PENDING, TARGET_REACHED, EARLY_CLOSE, NO_PROGRESS, TIMEOUT };

  explicit SoftClawMotion(const SoftClawMotionConfig& config) : config_(config) {}

  void start(double initial_position, double target_position, TimePoint now)
  {
    initial_position_ = initial_position;
    target_position_ = target_position;
    started_at_ = now;
    latest_position_ = initial_position;
    max_close_progress_ = 0.0;
    target_sample_count_ = 0;
    target_since_ = TimePoint();
    recent_samples_.clear();
    active_ = true;
  }

  void reset()
  {
    active_ = false;
    recent_samples_.clear();
  }

  bool active() const { return active_; }
  double target() const { return target_position_; }

  void observe(double position, TimePoint now)
  {
    if (!active_ || !std::isfinite(position) || now < started_at_)
      return;

    latest_position_ = position;
    max_close_progress_ = std::max(max_close_progress_, position - initial_position_);

    if (std::abs(position - target_position_) <= config_.position_tolerance)
    {
      if (target_sample_count_ == 0)
        target_since_ = now;
      ++target_sample_count_;
    }
    else
    {
      target_sample_count_ = 0;
    }

    recent_samples_.push_back({now, position});
    const auto cutoff = now - seconds(config_.early_stability_sec);
    // Keep one sample at or before the start of the window so its span is measurable.
    while (recent_samples_.size() > 1 && recent_samples_[1].time <= cutoff)
      recent_samples_.pop_front();
  }

  Result evaluate(TimePoint now) const
  {
    if (!active_)
      return Result::PENDING;

    const bool closing = target_position_ > initial_position_ + config_.position_tolerance;
    if (elapsed(now, started_at_) >= config_.motion_timeout_sec)
      return closing && max_close_progress_ < config_.min_close_progress
                 ? Result::NO_PROGRESS : Result::TIMEOUT;

    if (target_sample_count_ >= 2 &&
        !recent_samples_.empty() &&
        elapsed(recent_samples_.back().time, target_since_) >= config_.target_stability_sec &&
        std::abs(latest_position_ - target_position_) <= config_.position_tolerance)
      return Result::TARGET_REACHED;

    if (closing && max_close_progress_ >= config_.min_close_progress &&
        latest_position_ < target_position_ - config_.position_tolerance &&
        recent_samples_.size() >= 3 &&
        elapsed(recent_samples_.back().time, recent_samples_.front().time) >= config_.early_stability_sec)
    {
      double min_position = std::numeric_limits<double>::infinity();
      double max_position = -std::numeric_limits<double>::infinity();
      for (const auto& sample : recent_samples_)
      {
        min_position = std::min(min_position, sample.position);
        max_position = std::max(max_position, sample.position);
      }
      if (max_position - min_position <= config_.stability_delta)
        return Result::EARLY_CLOSE;
    }

    return Result::PENDING;
  }

private:
  struct Sample
  {
    TimePoint time;
    double position;
  };

  static Clock::duration seconds(double value)
  {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(value));
  }

  static double elapsed(TimePoint end, TimePoint start)
  {
    return std::chrono::duration<double>(end - start).count();
  }

  SoftClawMotionConfig config_;
  bool active_ = false;
  double initial_position_ = 0.0;
  double target_position_ = 0.0;
  double latest_position_ = 0.0;
  double max_close_progress_ = 0.0;
  TimePoint started_at_;
  TimePoint target_since_;
  std::size_t target_sample_count_ = 0;
  std::deque<Sample> recent_samples_;
};

}  // namespace rpwc_bridge
