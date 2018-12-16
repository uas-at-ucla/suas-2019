#include <memory>

#include "src/controls/ground_server/timeline/timeline_grammar.pb.h"
#include "src/controls/ground_server/timeline/drone_state_machine/drone_state_machine.hh"
#include "src/controls/ground_server/timeline/drone_state_machine/states.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace drone2state_visitor {

using src::controls::ground_server::drone_state_machine::DroneStateMachine;
namespace states = src::controls::ground_server::drone_state_machine::states;
namespace timeline = src::controls::ground_server::timeline;

class Drone2StateVisitor {
 public:
  DroneStateMachine Process(const timeline::DroneProgram& drone_program);
};

} // namespace drone2state_visitor

} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
