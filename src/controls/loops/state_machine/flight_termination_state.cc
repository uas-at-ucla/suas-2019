#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {

FlightTerminationState::FlightTerminationState() {}

void FlightTerminationState::Handle(::src::controls::Sensors &sensors,
                                    ::src::controls::Goal &goal,
                                    ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void FlightTerminationState::Reset() {}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src