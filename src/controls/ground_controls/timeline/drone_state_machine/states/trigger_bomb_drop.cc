#include "trigger_bomb_drop.hh"

namespace src {
namespace controls {
namespace ground_controls {
namespace drone_state_machine {
namespace states {

TriggerBombDrop::TriggerBombDrop() { this->name_ = "TriggerBombDrop"; }

const std::vector<BranchId> TriggerBombDrop::ListBranches() const {
  return {NEXT};
}

Result TriggerBombDrop::Step(DroneContext ctx) {
  ctx.Output().set_bomb_drop(true);
  return Branch(NEXT);
}

} // namespace states
} // namespace drone_state_machine
} // namespace ground_controls
} // namespace controls
} // namespace src
