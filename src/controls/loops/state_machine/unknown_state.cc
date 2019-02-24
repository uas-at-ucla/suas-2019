#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {
UnknownState::UnknownState() {}

void UnknownState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;

  // LOG_LINE("Unknown state!");
}

void UnknownState::Reset() {}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src
