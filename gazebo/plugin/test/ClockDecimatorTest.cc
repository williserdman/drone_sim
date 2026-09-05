#include "../ClockDecimator.hh"

#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

int main()
{
  drone_sim::ClockDecimator decimator;
  const std::vector<std::pair<std::int64_t, bool>> cases{
    {-1, false},
    {0, true},
    {1'000'000, false},
    {49'000'000, false},
    {50'000'000, true},
    {50'000'000, false},
    {51'000'000, false},
    {100'000'000, true},
    {50'000'000, false},
  };
  for (const auto & [stamp, expected] : cases)
  {
    const bool actual = decimator.ShouldPublish(stamp);
    if (actual != expected)
    {
      std::cerr << "stamp " << stamp << " expected " << expected
                << " got " << actual << '\n';
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}
