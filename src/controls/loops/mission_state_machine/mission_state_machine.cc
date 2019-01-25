#include "mission_state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace mission_state_machine {

MissionStateMachine::MissionStateMachine() :
    state_(GET_NEXT_CMD),
    unknown_state_(new UnknownState()) {
  state_handlers_[TRANSLATE] = new TranslateState();
  state_handlers_[GET_NEXT_CMD] = new GetNextCmdState();
  state_handlers_[UGV_RELEASE] = new UGVReleaseState();
}

MissionStateMachine::~MissionStateMachine() {
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
  LOG_LINE("Running flight loop iteration @ "
           << ::std::fixed << ::std::setw(8) << ::std::setprecision(3)
           << sensors.time() << " with state: " << StateToString(state_));

  // Bypass current state if a safety signal is received.
  if (SafetyStateOverride(goal, output)) {
    StateTransition(output);
  }

  // Route to the correct state.
  LOG_LINE("Routing to state: " + StateToString(state_));
  GetStateHandler(state_)->Handle(sensors, goal, output);

  StateTransition(output);
}

void MissionStateMachine::StateTransition(::src::controls::Output &output) {
  MissionState old_state = state_;
  MissionState new_state = static_cast<MissionState>(output.state());

  if (old_state != new_state) {
    // Handle state transitions.
    LOG_LINE("Switching states: " << StateToString(old_state) << " -> "
                                  << StateToString(new_state));
  }

  state_ = new_state;
}

bool MissionStateMachine::SafetyStateOverride(::src::controls::Goal &goal,
                                              ::src::controls::Output &output) {
  (void)goal;
  (void)output;
  return false;
}

State *MissionStateMachine::GetStateHandler(MissionState state) {
  if (state_handlers_.count(state)) {
    return state_handlers_[state];
  }

  return unknown_state_;
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

} // namespace mission_state_machine
} // namespace loops
} // namespace controls
} // namespace src
