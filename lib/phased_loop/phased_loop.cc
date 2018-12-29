#include "phased_loop.h"

#include <iostream>

namespace lib {
namespace phased_loop {

double GetCurrentTime() {
  return ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
             ::std::chrono::steady_clock::now().time_since_epoch())
             .count() *
         1e-9;
}

PhasedLoop::PhasedLoop(double frequency) :
    frequency_(frequency),
    next_iteration_(GetCurrentTime()) {}

void PhasedLoop::SleepUntilNext() {
  double now = GetCurrentTime();
  double diff = next_iteration_ - now;
  next_iteration_ += 1.0 / frequency_;

  if (diff <= 0) {
    return;
  }

  ::std::this_thread::sleep_for(::std::chrono::duration<double>(diff));
}

} // namespace phased_loop
} // namespace lib
