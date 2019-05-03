#include "drone_state.hh"

namespace src {
namespace controls {
namespace ground_controls {
namespace drone_state_machine {

const controls::Sensors &DroneContext::Sensors() { return this->sensors_; }

controls::Output &DroneContext::Output() { return this->output_; }

} // namespace drone_state_machine
} // namespace ground_controls
} // namespace controls
} // namespace src
