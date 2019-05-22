#include "state_machine.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace state_machine {

FailsafeState::FailsafeState() {}

void FailsafeState::Handle(::src::controls::Sensors &sensors,
                           ::src::controls::Goal &goal,
                           ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void FailsafeState::Reset() {}

} // namespace state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src
