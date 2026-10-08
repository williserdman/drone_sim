#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>

#include <gz/msgs/imu.pb.h>

namespace drone_sim::gazebo
{
enum class ImuWaitResult
{
  Matched,
  TimedOut,
  FutureSample,
  Cancelled,
};

struct ImuWaitOutcome
{
  ImuWaitResult result{ImuWaitResult::TimedOut};
  gz::msgs::IMU sample;
  std::int64_t timestampNs{-1};
};

class ImuSampleSynchronizer
{
public:
  void Publish(const gz::msgs::IMU &_sample)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->latest = _sample;
      this->valid = true;
    }
    this->condition.notify_all();
  }

  ImuWaitOutcome WaitFor(
      std::int64_t _expectedTimestampNs,
      std::chrono::milliseconds _timeout)
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    const bool ready = this->condition.wait_for(
        lock, _timeout, [this, _expectedTimestampNs]
        {
          return this->cancelled ||
              (this->valid &&
              TimestampNs(this->latest) >= _expectedTimestampNs);
        });
    if (this->cancelled)
      return {ImuWaitResult::Cancelled, {}, -1};

    const std::int64_t observed = this->valid ? TimestampNs(this->latest) : -1;
    if (!ready)
      return {ImuWaitResult::TimedOut, {}, observed};
    if (observed != _expectedTimestampNs)
      return {ImuWaitResult::FutureSample, {}, observed};
    return {ImuWaitResult::Matched, this->latest, observed};
  }

  void Cancel()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->cancelled = true;
    }
    this->condition.notify_all();
  }

  void Reset()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->latest.Clear();
    this->valid = false;
    this->cancelled = false;
  }

private:
  static std::int64_t TimestampNs(const gz::msgs::IMU &_sample)
  {
    const auto &stamp = _sample.header().stamp();
    return static_cast<std::int64_t>(stamp.sec()) * 1'000'000'000LL +
        static_cast<std::int64_t>(stamp.nsec());
  }

  std::mutex mutex;
  std::condition_variable condition;
  gz::msgs::IMU latest;
  bool valid{false};
  bool cancelled{false};
};
}  // namespace drone_sim::gazebo
