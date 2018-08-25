#include "phased_loop.h"

namespace lib {
namespace phased_loop {

PhasedLoop::PhasedLoop(double frequency)
    : frequency_(frequency), next_iteration_(GetCurrentTime()) {}

void PhasedLoop::SleepUntilNext() {
  double now = GetCurrentTime();
  double diff = next_iteration_ - now;

  if (diff <= 0) {
    return;
  }

  next_iteration_ = now + 1.0 / frequency_;

  ::std::this_thread::sleep_for(::std::chrono::duration<double>(diff));
}

double PhasedLoop::GetCurrentTime() {
  return ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
             ::std::chrono::system_clock::now().time_since_epoch())
             .count() *
         1e-9;
}

} // namespace phased_loop
} // namespace lib
