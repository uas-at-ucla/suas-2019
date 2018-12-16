#include "src/controls/ground_server/timeline/state_machine/branching_state.hh"

#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {

namespace controls = ::src::controls;

namespace state_machine = src::controls::ground_server::state_machine;
namespace result = state_machine::result;

using state_machine::BranchId;
using state_machine::StateId;
using result::Result;

class DroneContext {
public:
  const controls::Sensors& Sensors();
  controls::Output& Output();

private:
  controls::Sensors sensors_;
  controls::Output output_;
};

typedef state_machine::State<DroneContext> DroneState;
typedef state_machine::BranchingState<DroneContext> BranchingDroneState;

} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
