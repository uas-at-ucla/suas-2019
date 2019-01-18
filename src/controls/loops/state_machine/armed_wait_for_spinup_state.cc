#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {
ArmedWaitForSpinupState::ArmedWaitForSpinupState() :
    start_(::std::numeric_limits<double>::infinity()) {}

void ArmedWaitForSpinupState::Handle(::src::controls::Sensors &sensors,
                                     ::src::controls::Goal &goal,
                                     ::src::controls::Output &output) {
  (void)goal;

  // Set initial time if it was not set yet.
  if (start_ == ::std::numeric_limits<double>::infinity()) {
    start_ = sensors.time();
  }

  // Wait a bit for the propellers to spin up while armed.
  if (sensors.time() - start_ >= kSpinupTime) {
    output.set_state(ARMED);
  }
}

void ArmedWaitForSpinupState::Reset() {
  start_ = ::std::numeric_limits<double>::infinity();
}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src
