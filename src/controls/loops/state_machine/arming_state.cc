#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {

ArmingState::ArmingState() {}

void ArmingState::Handle(::src::controls::Sensors &sensors,
                         ::src::controls::Goal &goal,
                         ::src::controls::Output &output) {
  (void)goal;

  // Check if we have GPS.
  if (sensors.last_gps() < sensors.time() - 0.5) {
    LOG_LINE("can't arm; no GPS "
             << "(last gps: " << sensors.last_gps()
             << " current time: " << sensors.time());

    output.set_state(STANDBY);
  }

  if (sensors.armed()) {
    output.set_state(ARMED_WAIT_FOR_SPINUP);
  }

  output.set_trigger_arm(sensors.time());
}

void ArmingState::Reset() {}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src
