#include "state_machine.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace state_machine {

TakenOffState::TakenOffState() {}

void TakenOffState::Handle(::src::controls::Sensors &sensors,
                           ::src::controls::Goal &goal,
                           ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;

  if (goal.run_mission()) {
    output.set_state(ARMED);
  }
}

void TakenOffState::Reset() {}

} // namespace state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src
