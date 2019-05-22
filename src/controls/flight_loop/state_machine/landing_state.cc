#include "state_machine.h"

namespace src {
namespace controls {
namespace flight_loop {
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

void LandingState::Reset() {}

} // namespace state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src
