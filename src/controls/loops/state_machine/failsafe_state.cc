#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
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
} // namespace loops
} // namespace controls
} // namespace src
