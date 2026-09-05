#include "ClockDecimator.hh"

namespace drone_sim
{

bool ClockDecimator::ShouldPublish(const std::int64_t stampNanoseconds)
{
  if (stampNanoseconds < 0 ||
      stampNanoseconds % kPeriodNanoseconds != 0 ||
      stampNanoseconds <= this->lastPublishedNanoseconds)
  {
    return false;
  }
  this->lastPublishedNanoseconds = stampNanoseconds;
  return true;
}

}  // namespace drone_sim
