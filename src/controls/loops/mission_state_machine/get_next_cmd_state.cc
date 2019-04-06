#include "mission_state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace mission_state_machine {

GetNextCmdState::GetNextCmdState() {}

void GetNextCmdState::Handle(::src::controls::Sensors &sensors,
                             ::src::controls::Goal &goal,
                             ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void GetNextCmdState::Reset() {}

} // namespace mission_state_machine
} // namespace loops
} // namespace controls
} // namespace src
