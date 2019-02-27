#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {

StandbyState::StandbyState() {}

void StandbyState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  if (goal.run_mission()) {
    //  LOG_LINE("Run mission requested; attempting to arm.");
    output.set_state(ARMING);
  }

  if (sensors.armed()) {
    //  LOG_LINE("Pixhawk is armed; switching to ARMED state.");
    output.set_state(ARMED);
  }
}

void StandbyState::Reset() {}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src
