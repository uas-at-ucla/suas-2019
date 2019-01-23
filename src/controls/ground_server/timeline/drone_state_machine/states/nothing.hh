#pragma once

#include "../drone_state_machine.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {
namespace states {

using namespace src::controls::ground_server::drone_state_machine;

class Nothing : public BranchingDroneState {
 public:
  constexpr static BranchId NEXT = 1;

  Nothing();

  virtual const std::vector<BranchId> ListBranches() const override;

 protected:
  Result Step(DroneContext ctx) override;
};

} // namespace states
} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
