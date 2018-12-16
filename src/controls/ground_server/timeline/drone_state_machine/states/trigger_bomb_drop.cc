#include "trigger_bomb_drop.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {
namespace states {

TriggerBombDrop::TriggerBombDrop() {
  this->name_ = "TriggerBombDrop";
}

Result TriggerBombDrop::Step(DroneContext ctx) {
  ctx.Output().set_bomb_drop(true);
  return Branch(NEXT);
}

} // namespace states
} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
