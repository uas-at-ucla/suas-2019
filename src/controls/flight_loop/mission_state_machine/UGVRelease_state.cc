#include "mission_state_machine.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace mission_state_machine {

UGVReleaseState::UGVReleaseState() {}

void UGVReleaseState::Handle(::src::controls::Sensors &sensors,
                             ::src::controls::Goal &goal,
                             ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void UGVReleaseState::Reset() {}

} // namespace mission_state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src
