#include "nothing.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {
namespace states {

Nothing::Nothing() {
  this->name_ = "Nothing";
}

Result Nothing::Step(DroneContext ctx) {
  (void)ctx;
  return result::FINISHED;
}

} // namespace states
} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
