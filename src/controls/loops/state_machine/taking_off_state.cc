#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {

TakingOffState::TakingOffState() {}

void TakingOffState::Handle(::src::controls::Sensors &sensors,
                            ::src::controls::Goal &goal,
                            ::src::controls::Output &output) {

  // Arm the drone before performing a takeoff.
  if (!sensors.armed()) {
    output.set_state(ARMING);
    return;
  }

  // Ensure that the drone reaches a safe altitude before going into the next
  // state.
  if (sensors.relative_altitude() > kTakeoffAltitude) {
    output.set_state(TAKEN_OFF);
  }
}

void TakingOffState::Reset() {}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src
