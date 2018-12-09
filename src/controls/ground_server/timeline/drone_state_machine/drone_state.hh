#include "src/controls/ground_server/timeline/state_machine/branching_state.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {

namespace state_machine = src::controls::ground_server::state_machine;
namespace result = state_machine::result;

using state_machine::BranchId;
using state_machine::StateId;
using result::Result;

class DroneContext {};

typedef state_machine::State<DroneContext> DroneState;
typedef state_machine::BranchingState<DroneContext> BranchingDroneState;

class NothingState : public BranchingDroneState {};

class SleepState : public BranchingDroneState {};

class GotoState : public BranchingDroneState {};

class TranslateState : public BranchingDroneState {};

class TriggerBombDropState : public BranchingDroneState {};

class TriggerAlarmState : public BranchingDroneState {};

} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
