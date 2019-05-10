#pragma once

#include "src/controls/ground_controls/timeline/state_machine/state_machine.hh"

#include "drone_state.hh"

namespace src {
namespace controls {
namespace ground_controls {
namespace drone_state_machine {

typedef state_machine::StateMachine<DroneContext> DroneStateMachine;

} // namespace drone_state_machine
} // namespace ground_controls
} // namespace controls
} // namespace src
