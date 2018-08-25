#pragma once

#include <chrono>
#include <thread>

namespace lib {
namespace phased_loop {

class PhasedLoop {
 public:
  PhasedLoop(double frequency);
  void SleepUntilNext();

 private:
  double GetCurrentTime();

  double frequency_;
  double next_iteration_;
};

} // namespace phased_loop
} // namespace lib
