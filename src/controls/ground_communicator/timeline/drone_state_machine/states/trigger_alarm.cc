#include "trigger_alarm.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace drone_state_machine {
namespace states {

TriggerAlarm::TriggerAlarm() { this->name_ = "TriggerAlarm"; }

const std::vector<BranchId> TriggerAlarm::ListBranches() const {
  return {NEXT};
}

Result TriggerAlarm::Step(DroneContext ctx) {
  ctx.Output().set_alarm(true);
  return Branch(NEXT);
}

} // namespace states
} // namespace drone_state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
