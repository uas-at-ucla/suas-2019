#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {
LandingState::LandingState() {}

void LandingState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {

  if (!sensors.armed()) {
    // EndFlightTimer();
    output.set_state(STANDBY);
    return;
  }

  if (goal.run_mission() && sensors.relative_altitude() > 5.0) {
    output.set_state(MISSION);
  }
}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src