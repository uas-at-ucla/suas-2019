#include <memory>

#include "src/controls/ground_server/timeline/drone_state_machine/drone_state_machine.hh"
#include "src/controls/ground_server/timeline/drone_state_machine/states.hh"
#include "src/controls/ground_server/timeline/timeline_grammar.pb.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace drone2state_visitor {

using namespace src::controls::ground_server::drone_state_machine;
namespace timeline = src::controls::ground_server::timeline;

class Drone2StateVisitor {
 public:
  DroneStateMachine Process(const timeline::DroneProgram &drone_program);

 private:
  constexpr static StateId initial_state_id_ = 1;

  DroneStateMachine::States states_;
  std::shared_ptr<BranchingDroneState> last_branching_state_;
  BranchId last_branch_id_;
  StateId next_state_id_;

  void AddState(DroneStateMachine::StatePtr state);
  void AddBranchingState(std::shared_ptr<BranchingDroneState> branching_state,
                         BranchId branch_id);

  template <typename T> std::shared_ptr<T> MakeBranchingState();

  template <typename T>
  std::shared_ptr<T> MakeBranchingState(BranchId branch_id);

  void Visit(const timeline::DroneProgram &drone_program);
  void Visit(const timeline::DroneCommand &drone_command);
  void Visit(const lib::mission_manager::NothingCommand &drone_command);
  void Visit(const lib::mission_manager::SleepCommand &drone_command);
  void Visit(const timeline::TranslateCommand &drone_command);
  void Visit(const timeline::TriggerAlarmCommand &drone_command);
  void Visit(const timeline::TriggerBombDropCommand &drone_command);
};

class UnsupportedDroneCommandException : public std::exception {
 public:
  UnsupportedDroneCommandException(const timeline::DroneCommand &command);

  virtual const char *what() const noexcept override;

  const timeline::DroneCommand &Command() const;

 private:
  timeline::DroneCommand command_;
  std::string message_;
};

template <typename T>
std::shared_ptr<T> Drone2StateVisitor::MakeBranchingState() {
  return MakeBranchingState<T>(T::NEXT);
}

template <typename T>
std::shared_ptr<T> Drone2StateVisitor::MakeBranchingState(BranchId branch_id) {
  auto state = std::make_shared<T>();
  AddBranchingState(state, branch_id);
  return state;
}

} // namespace drone2state_visitor

} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
