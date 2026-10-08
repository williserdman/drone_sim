#include "../ImuSampleSynchronizer.hh"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <iostream>
#include <string>

namespace
{
using namespace std::chrono_literals;
using drone_sim::gazebo::ImuSampleSynchronizer;
using drone_sim::gazebo::ImuWaitResult;

void Require(bool _condition, const std::string &_message)
{
  if (!_condition)
  {
    std::cerr << _message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

gz::msgs::IMU Sample(
    std::int64_t _timestampNs, double _x, double _y, double _z)
{
  gz::msgs::IMU sample;
  sample.mutable_header()->mutable_stamp()->set_sec(_timestampNs / 1'000'000'000);
  sample.mutable_header()->mutable_stamp()->set_nsec(_timestampNs % 1'000'000'000);
  sample.mutable_linear_acceleration()->set_x(_x);
  sample.mutable_linear_acceleration()->set_y(_y);
  sample.mutable_linear_acceleration()->set_z(_z);
  return sample;
}

void TestDelayedCurrentSampleReplacesStaleSample()
{
  ImuSampleSynchronizer synchronizer;
  synchronizer.Publish(Sample(117'366'000'000, 0.0, 0.0, -9.8));

  auto waiting = std::async(std::launch::async, [&synchronizer]
  {
    return synchronizer.WaitFor(117'367'000'000, 500ms);
  });
  Require(waiting.wait_for(20ms) == std::future_status::timeout,
      "a stale sample must not satisfy the current simulation step");

  synchronizer.Publish(Sample(
      117'367'000'000, 9.612777, 219.458916, -258.747801));
  const auto first = waiting.get();
  Require(first.result == ImuWaitResult::Matched,
      "the delayed current sample must satisfy the waiter");
  Require(first.timestampNs == 117'367'000'000,
      "the matched sample must keep its exact integer timestamp");
  Require(std::abs(first.sample.linear_acceleration().y() - 219.458916) < 1e-9,
      "the first impulse must be returned unchanged");

  synchronizer.Publish(Sample(
      117'368'000'000, -8.482831, -222.404311, -261.863924));
  const auto second = synchronizer.WaitFor(117'368'000'000, 20ms);
  Require(second.result == ImuWaitResult::Matched,
      "the next opposing impulse must also match once at its own step");
  Require(second.timestampNs == 117'368'000'000,
      "the second impulse must not reuse the prior timestamp");
  Require(std::abs(
      first.sample.linear_acceleration().y() +
      second.sample.linear_acceleration().y() + 2.945395) < 1e-6,
      "both opposing lateral impulses must reach the consumer unchanged");
}

void TestTimeoutAndFutureSampleFailClosed()
{
  ImuSampleSynchronizer synchronizer;
  synchronizer.Publish(Sample(1'000'000, 0.0, 0.0, -9.8));
  const auto timeout = synchronizer.WaitFor(2'000'000, 1ms);
  Require(timeout.result == ImuWaitResult::TimedOut,
      "an old sample must time out instead of masquerading as fresh");
  Require(timeout.timestampNs == 1'000'000,
      "timeout diagnostics must retain the observed timestamp");

  synchronizer.Publish(Sample(3'000'000, 0.0, 0.0, -9.8));
  const auto future = synchronizer.WaitFor(2'000'000, 20ms);
  Require(future.result == ImuWaitResult::FutureSample,
      "a future sample must fail immediately instead of satisfying an old step");
  Require(future.timestampNs == 3'000'000,
      "future-sample diagnostics must retain the observed timestamp");
}

void TestCancelAndResetReleaseWaitersWithoutReusingState()
{
  ImuSampleSynchronizer synchronizer;
  auto waiting = std::async(std::launch::async, [&synchronizer]
  {
    return synchronizer.WaitFor(4'000'000, 500ms);
  });
  Require(waiting.wait_for(20ms) == std::future_status::timeout,
      "the empty synchronizer must wait for the requested sample");
  synchronizer.Cancel();
  Require(waiting.get().result == ImuWaitResult::Cancelled,
      "cancellation must wake an active waiter");

  synchronizer.Reset();
  synchronizer.Publish(Sample(5'000'000, 1.0, 2.0, 3.0));
  const auto current = synchronizer.WaitFor(5'000'000, 20ms);
  Require(current.result == ImuWaitResult::Matched,
      "reset must clear cancellation and permit a new exact sample");

  synchronizer.Reset();
  const auto cleared = synchronizer.WaitFor(5'000'000, 1ms);
  Require(cleared.result == ImuWaitResult::TimedOut && cleared.timestampNs == -1,
      "reset must clear the previously matched sample");
}
}

int main()
{
  TestTimeoutAndFutureSampleFailClosed();
  TestDelayedCurrentSampleReplacesStaleSample();
  TestCancelAndResetReleaseWaitersWithoutReusingState();
  return EXIT_SUCCESS;
}
