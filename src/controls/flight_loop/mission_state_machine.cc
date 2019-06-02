#include "mission_state_machine.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace mission_state_machine {

MissionStateMachine::MissionStateMachine() :
    state_(GET_NEXT_CMD),
    unknown_state_(new UnknownState()) {

  // Create an instance of all state handlers.
  state_handlers_[TRANSLATE] = new TranslateState();
  state_handlers_[GET_NEXT_CMD] = new GetNextCmdState();
  state_handlers_[UGV_RELEASE] = new UGVReleaseState();
}

MissionStateMachine::~MissionStateMachine() {
  // Delete all state handler instances.
  for (auto const &state_handler_pair : state_handlers_) {
    delete state_handler_pair.second;
  }

  delete unknown_state_;
}

void MissionStateMachine::Handle(::src::controls::Sensors &sensors,
                                 ::src::controls::Goal &goal,
                                 ::src::controls::Output &output) {

  // Use same state in next loop iteration, unless it is changed.
  output.set_state(state_);

  // Handle current state.
  GetStateHandler(state_)->Handle(sensors, goal, output);

  // Transition to next state.
  StateTransition(output);
}

void MissionStateMachine::StateTransition(::src::controls::Output &output) {
  MissionState old_state = state_;
  MissionState new_state = static_cast<MissionState>(output.state());

  // Do nothing if state does not change.
  if (old_state == new_state) {
    return;
  }

  // Log all state transitions.
  ROS_INFO("Switching states: %s -> %s", StateToString(old_state).c_str(),
           StateToString(new_state).c_str());

  // Apply the state transition.
  state_ = new_state;

  // Reset the new state after transition is made.
  GetStateHandler(state_)->Reset();
}

State *MissionStateMachine::GetStateHandler(MissionState state) {
  if (state_handlers_.count(state)) {
    return state_handlers_[state];
  }

  return unknown_state_;
}

void MissionStateMachine::LoadMission(
    ::src::controls::ground_controls::timeline::DroneProgram drone_program) {
  (void)drone_program;
}

::std::string StateToString(MissionState state) {
  switch (state) {
    case TRANSLATE:
      return "TRANSLATE";
    case UGV_RELEASE:
      return "UGV_RELEASE";
    case GET_NEXT_CMD:
      return "GET_NEXT_CMD";
  }
  return "UNKNOWN";
}

// GetNextCmdState /////////////////////////////////////////////////////////////
GetNextCmdState::GetNextCmdState() {}

void GetNextCmdState::Handle(::src::controls::Sensors &sensors,
                             ::src::controls::Goal &goal,
                             ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void GetNextCmdState::Reset() {}

// TranslateState //////////////////////////////////////////////////////////////
TranslateState::TranslateState() {}

void TranslateState::Handle(::src::controls::Sensors &sensors,
                            ::src::controls::Goal &goal,
                            ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void TranslateState::Reset() {}

// UGVReleaseState /////////////////////////////////////////////////////////////
UGVReleaseState::UGVReleaseState() {}

void UGVReleaseState::Handle(::src::controls::Sensors &sensors,
                             ::src::controls::Goal &goal,
                             ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void UGVReleaseState::Reset() {}

// UGVReleaseState /////////////////////////////////////////////////////////////
UnknownState::UnknownState() {}

void UnknownState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;

  ROS_ERROR("UNKNOWN STATE!!!");
}

void UnknownState::Reset() {}

} // namespace mission_state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src
