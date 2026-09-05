#pragma once

#include <cstdint>

namespace drone_sim
{

class ClockDecimator
{
public:
  static constexpr std::int64_t kPeriodNanoseconds = 50'000'000;

  bool ShouldPublish(std::int64_t stampNanoseconds);

private:
  std::int64_t lastPublishedNanoseconds{-1};
};

}  // namespace drone_sim
