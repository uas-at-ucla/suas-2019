#include "trigger.h"

namespace lib {
namespace trigger {

Trigger::Trigger(double tolerance) :
    tolerance_(tolerance),
    did_trigger_(false) {}

bool Trigger::Process(double trigger_time) {
  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  if (trigger_time + tolerance_ > current_time) {
    if (!did_trigger_) {
      did_trigger_ = true;
      return true;
    }

    did_trigger_ = false;
  }

  return false;
}

} // namespace trigger
} // namespace lib
