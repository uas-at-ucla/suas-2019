#include "drone_state.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {

const controls::Sensors& DroneContext::Sensors() {
  return this->sensors_;
}

controls::Output& DroneContext::Output() {
  return this->output_;
}

}
} // namespace ground_server
} // namespace controls
} // namespace src
