#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {

TakingOffState::TakingOffState() {}

void TakingOffState::Handle(::src::controls::Sensors &sensors,
                            ::src::controls::Goal &goal,
                            ::src::controls::Output &output) {
  if (!goal.run_mission()) {
    // takeoff_ticker_ = 0;
    output.set_state(LANDING);
    return;
  }

  if (!sensors.armed()) {
    // takeoff_ticker_ = 0;
    output.set_state(ARMING);
    return;
  }

  if (sensors.relative_altitude() < 0.3) {
    // takeoff_ticker_++;
  }

  if (sensors.relative_altitude() > 2.2) {
    // takeoff_ticker_ = 0;
    output.set_state(TAKEN_OFF);
  }
}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src