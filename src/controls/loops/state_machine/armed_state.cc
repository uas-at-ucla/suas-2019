#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {

ArmedState::ArmedState() {}

void ArmedState::Handle(::src::controls::Sensors &sensors,
                        ::src::controls::Goal &goal,
                        ::src::controls::Output &output) {
  // current_flight_start_time_ = ::lib::phased_loop::GetCurrentTime();

  if (!sensors.armed()) {
    if (goal.run_mission()) {
      output.set_state(ARMING);
    } else {
      output.set_state(STANDBY);
    }
  }
}

void ArmedState::Reset() {}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src
