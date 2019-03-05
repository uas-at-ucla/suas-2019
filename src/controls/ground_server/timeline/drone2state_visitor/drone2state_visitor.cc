#include "drone2state_visitor.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace drone2state_visitor {

DroneStateMachine
Drone2StateVisitor::Process(const timeline::DroneProgram &program) {
  states_.clear();
  next_state_id_ = initial_state_id_;
  last_branching_state_.reset();

  Visit(program);

  return DroneStateMachine(states_, initial_state_id_);
}

void Drone2StateVisitor::AddState(DroneStateMachine::StatePtr state) {
  const StateId state_id = next_state_id_++;
  if (last_branching_state_) {
    last_branching_state_->SetBranch(last_branch_id_, state_id);
    last_branching_state_.reset();
  }
  states_[state_id] = state;
}

void Drone2StateVisitor::AddBranchingState(
    std::shared_ptr<BranchingDroneState> branching_state, BranchId branch_id) {
  AddState(branching_state);
  last_branching_state_ = branching_state;
  last_branch_id_ = branch_id;
}

void Drone2StateVisitor::Visit(const timeline::DroneProgram &drone_program) {
  const auto &commands = drone_program.commands();
  for (const timeline::DroneCommand &command : commands) {
    Visit(command);
  }
}

void Drone2StateVisitor::Visit(const timeline::DroneCommand &drone_command) {
  switch (drone_command.command_case()) {
    case timeline::DroneCommand::kNothingCommand:
      Visit(drone_command.nothing_command());
      break;
    case timeline::DroneCommand::kSleepCommand:
      Visit(drone_command.sleep_command());
      break;
    case timeline::DroneCommand::kTranslateCommand:
      Visit(drone_command.translate_command());
      break;
    case timeline::DroneCommand::kTriggerAlarmCommand:
      Visit(drone_command.trigger_alarm_command());
      break;
    case timeline::DroneCommand::kTriggerBombDropCommand:
      Visit(drone_command.trigger_bomb_drop_command());
      break;
    default:
      throw UnsupportedDroneCommandException(drone_command);
  }
}

void Drone2StateVisitor::Visit(
    const ::lib::mission_manager::NothingCommand &command) {
  (void)command;
  MakeBranchingState<states::Nothing>();
}

void Drone2StateVisitor::Visit(
    const ::lib::mission_manager::SleepCommand &command) {
  auto sleep_state = MakeBranchingState<states::Sleep>();
  sleep_state->SetDurationSecs(command.time());
}

void Drone2StateVisitor::Visit(const timeline::TranslateCommand &command) {
  auto translate_state = MakeBranchingState<states::Translate>();
  (void)command;
  (void)translate_state;
  // TODO: actually set up states::Translate
}

void Drone2StateVisitor::Visit(const timeline::TriggerAlarmCommand &command) {
  (void)command;
  MakeBranchingState<states::TriggerAlarm>();
}

void Drone2StateVisitor::Visit(
    const timeline::TriggerBombDropCommand &command) {
  (void)command;
  MakeBranchingState<states::TriggerBombDrop>();
}

UnsupportedDroneCommandException::UnsupportedDroneCommandException(
    const timeline::DroneCommand &command) {
  command_ = command;
  message_ = "Unsupported drone command: " + command_.DebugString();
}

const char *UnsupportedDroneCommandException::what() const noexcept {
  return message_.c_str();
}

const timeline::DroneCommand &
UnsupportedDroneCommandException::Command() const {
  return command_;
}

} // namespace drone2state_visitor
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src