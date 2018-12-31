#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {
ArmedWaitForSpinupState::ArmedWaitForSpinupState() {}

void ArmedWaitForSpinupState::Handle(::src::controls::Sensors &sensors,
                                     ::src::controls::Goal &goal,
                                     ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;

  // Wait a bit for the propellers to spin up while armed.
  output.set_state(ARMED);
}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src