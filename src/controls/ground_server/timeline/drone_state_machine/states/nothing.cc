#include "nothing.hh"

#include <iostream>

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {
namespace states {

Nothing::Nothing() { this->name_ = "Nothing"; }

const std::vector<BranchId> Nothing::ListBranches() const { return {NEXT}; }

Result Nothing::Step(DroneContext ctx) {
  (void)ctx;
  std::cout << "running state Nothing" << std::endl;
  return Branch(NEXT);
}

} // namespace states
} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
