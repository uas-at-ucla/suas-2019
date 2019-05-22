#include "state_machine.h"

namespace src {
namespace controls {
namespace flight_loop {
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
} // namespace flight_loop
} // namespace controls
} // namespace src
