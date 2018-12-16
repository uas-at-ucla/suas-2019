#include "translate.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {
namespace states {

Translate::Translate() {
  this->name_ = "Translate";
}

Result Translate::Step(DroneContext ctx) {
  (void)ctx;
  // TODO: Implement command
  return Branch(NEXT);
}

} // namespace states
} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
