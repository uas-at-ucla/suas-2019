#include "drone2state_visitor.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace drone2state_visitor {

DroneStateMachine
Drone2StateVisitor::Process(const timeline::DroneProgram &program) {
  (void) program;
  return DroneStateMachine({}, 0);
}

} // namespace drone2state_visitor
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src