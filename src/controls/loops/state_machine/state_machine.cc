#include "state_machine.h"

namespace src {
namespace controls {
namespace loops {
namespace state_machine {

StateMachine::StateMachine() :
    state_(STANDBY),
    unknown_state_(new UnknownState()) {
  state_handlers_[STANDBY] = new StandbyState();
  state_handlers_[ARMING] = new ArmingState();
  state_handlers_[ARMED_WAIT_FOR_SPINUP] = new ArmedWaitForSpinupState();
  state_handlers_[ARMED] = new ArmedState();
  state_handlers_[TAKING_OFF] = new TakingOffState();
  state_handlers_[TAKEN_OFF] = new TakenOffState();
  state_handlers_[SAFETY_PILOT_CONTROL] = new SafetyPilotControlState();
  state_handlers_[MISSION] = new MissionState();
  state_handlers_[FAILSAFE] = new FailsafeState();
  state_handlers_[FLIGHT_TERMINATION] = new FlightTerminationState();
}

StateMachine::~StateMachine() {
  for (auto const &state_handler_pair : state_handlers_) {
    delete state_handler_pair.second;
  }

  delete unknown_state_;
}

void StateMachine::Handle(::src::controls::Sensors &sensors,
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

void StateMachine::StateTransition(::src::controls::Output &output) {
  FlightLoopState old_state = state_;
  FlightLoopState new_state = static_cast<FlightLoopState>(output.state());

  if (old_state != new_state) {
    // Handle state transitions.
    LOG_LINE("Switching states: " << StateToString(old_state) << " -> "
                                  << StateToString(new_state));
  }

  state_ = new_state;
}

bool StateMachine::SafetyStateOverride(::src::controls::Goal &goal,
                                       ::src::controls::Output &output) {
  // Prioritize the throttle cut check, so that the drone always cuts throttle
  // instead of following any failsafe commands.
  if (goal.trigger_throttle_cut()) {
    // EndFlightTimer();
    output.set_state(FLIGHT_TERMINATION);
    return true;
  }

  if (goal.trigger_failsafe()) {
    // EndFlightTimer();
    output.set_state(FAILSAFE);
    return true;
  }

  return false;
}

State *StateMachine::GetStateHandler(FlightLoopState state) {
  if (state_handlers_.count(state)) {
    return state_handlers_[state];
  }

  return unknown_state_;
}

::std::string StateToString(FlightLoopState state) {
  switch (state) {
    case STANDBY:
      return "STANDBY";
    case ARMING:
      return "ARMING";
    case ARMED_WAIT_FOR_SPINUP:
      return "ARMED_WAIT_FOR_SPINUP";
    case ARMED:
      return "ARMED";
    case TAKING_OFF:
      return "TAKING_OFF";
    case TAKEN_OFF:
      return "TAKEN_OFF";
    case SAFETY_PILOT_CONTROL:
      return "SAFETY_PILOT_CONTROL";
    case MISSION:
      return "MISSION";
    case LANDING:
      return "LANDING";
    case FAILSAFE:
      return "FAILSAFE";
    case FLIGHT_TERMINATION:
      return "FLIGHT_TERMINATION";
  }
  return "UNKNOWN";
}

} // namespace state_machine
} // namespace loops
} // namespace controls
} // namespace src
