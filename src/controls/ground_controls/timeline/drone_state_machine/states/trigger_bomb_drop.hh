#pragma once

#include "../drone_state_machine.hh"

namespace src {
namespace controls {
namespace ground_controls {
namespace drone_state_machine {
namespace states {

using namespace src::controls::ground_controls::drone_state_machine;

class TriggerBombDrop : public BranchingDroneState {
 public:
  constexpr static BranchId NEXT = 1;

  TriggerBombDrop();

  const std::vector<BranchId> ListBranches() const override;

 protected:
  Result Step(DroneContext ctx) override;
};

} // namespace states
} // namespace drone_state_machine
} // namespace ground_controls
} // namespace controls
} // namespace src
