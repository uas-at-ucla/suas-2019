#include "state_machine.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace state_machine {

StateMachine::StateMachine() :
    state_(STANDBY),
    unknown_state_(new UnknownState()) {

  // Create an instance of all state handlers.
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
  // Delete all state handler instances.
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

  // Bypass current state if a safety signal is received.
  if (SafetyStateOverride(goal, output)) {
    StateTransition(output);
  }

  // Handle current state.
  GetStateHandler(state_)->Handle(sensors, goal, output);
  StateTransition(output);
}

void StateMachine::StateTransition(::src::controls::Output &output) {
  // Record the old and new states.
  FlightLoopState old_state = state_;
  FlightLoopState new_state = static_cast<FlightLoopState>(output.state());

  // Log all state transitions.
  if (old_state != new_state) {
    ROS_INFO_STREAM("Switching states: " << StateToString(old_state) << " -> "
                                         << StateToString(new_state));
  }

  // Apply the state transition.
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

// ArmedState //////////////////////////////////////////////////////////////////
ArmedState::ArmedState() {}

void ArmedState::Handle(::src::controls::Sensors &sensors,
                        ::src::controls::Goal &goal,
                        ::src::controls::Output &output) {

  if (!sensors.armed()) {
    if (goal.run_mission()) {
      output.set_state(ARMING);
    } else {
      output.set_state(STANDBY);
    }

    return;
  }

  if (goal.run_mission()) {
    output.set_state(TAKING_OFF);
  }
}

void ArmedState::Reset() {}

// ArmingState /////////////////////////////////////////////////////////////////
ArmingState::ArmingState() {}

void ArmingState::Handle(::src::controls::Sensors &sensors,
                         ::src::controls::Goal &goal,
                         ::src::controls::Output &output) {
  (void)goal;

  // TODO(comran): Check that throttle is at zero before arming.

  // Check if we have GPS.
  if (sensors.last_gps() < sensors.time() - 0.5) {
    //  LOG_LINE("can't arm; no GPS "
    //           << "(last gps: " << sensors.last_gps()
    //           << " current time: " << sensors.time());

    output.set_state(STANDBY);
  }

  if (sensors.armed()) {
    output.set_state(ARMED_WAIT_FOR_SPINUP);
  }

  output.set_trigger_arm(sensors.time());
}

void ArmingState::Reset() {}

// ArmedWaitForSpinupState /////////////////////////////////////////////////////
ArmedWaitForSpinupState::ArmedWaitForSpinupState() :
    start_(::std::numeric_limits<double>::infinity()) {}

void ArmedWaitForSpinupState::Handle(::src::controls::Sensors &sensors,
                                     ::src::controls::Goal &goal,
                                     ::src::controls::Output &output) {
  (void)goal;

  // Set initial time if it was not set yet.
  if (start_ == ::std::numeric_limits<double>::infinity()) {
    start_ = sensors.time();
  }

  // Wait a bit for the propellers to spin up while armed.
  if (sensors.time() - start_ >= kSpinupTime) {
    output.set_state(ARMED);
  }
}

void ArmedWaitForSpinupState::Reset() {
  start_ = ::std::numeric_limits<double>::infinity();
}

// FailsafeState ///////////////////////////////////////////////////////////////
FailsafeState::FailsafeState() {}

void FailsafeState::Handle(::src::controls::Sensors &sensors,
                           ::src::controls::Goal &goal,
                           ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void FailsafeState::Reset() {}

// LandingState ////////////////////////////////////////////////////////////////
LandingState::LandingState() {}

void LandingState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {

  if (!sensors.armed()) {
    // EndFlightTimer();
    output.set_state(STANDBY);
    return;
  }

  if (goal.run_mission() && sensors.relative_altitude() > 5.0) {
    output.set_state(MISSION);
  }
}

void LandingState::Reset() {}

// FlightTerminationState //////////////////////////////////////////////////////
FlightTerminationState::FlightTerminationState() {}

void FlightTerminationState::Handle(::src::controls::Sensors &sensors,
                                    ::src::controls::Goal &goal,
                                    ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void FlightTerminationState::Reset() {}

// MissionState ////////////////////////////////////////////////////////////////
MissionState::MissionState() {}

void MissionState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  (void)goal;

  lib::Position3D position = {sensors.latitude(), sensors.longitude(),
                              sensors.relative_altitude()};

  ::Eigen::Vector3d velocity(sensors.velocity_x(), sensors.velocity_y(),
                             sensors.velocity_z());

  executor::ExecutorOutput executor_output =
      executor_.Calculate(position, velocity);

  // last_bomb_drop_ =
  //     executor_output.bomb_drop ? sensors.time() : last_bomb_drop_;

  if (executor_output.alarm) {
    // alarm_.AddAlert({5.0, 0.50});
  }

  output.set_velocity_x(executor_output.flight_velocities.x);
  output.set_velocity_y(executor_output.flight_velocities.y);
  output.set_velocity_z(executor_output.flight_velocities.z);
  output.set_yaw_setpoint(executor_output.yaw);
}

void MissionState::Reset() {}

// SafetyPilotControlState /////////////////////////////////////////////////////
SafetyPilotControlState::SafetyPilotControlState() {}

void SafetyPilotControlState::Handle(::src::controls::Sensors &sensors,
                                     ::src::controls::Goal &goal,
                                     ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

void SafetyPilotControlState::Reset() {}

// StandbyState ////////////////////////////////////////////////////////////////
StandbyState::StandbyState() {}

void StandbyState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  if (goal.run_mission()) {
    //  LOG_LINE("Run mission requested; attempting to arm.");
    output.set_state(ARMING);
  }

  if (sensors.armed()) {
    //  LOG_LINE("Pixhawk is armed; switching to ARMED state.");
    output.set_state(ARMED);
  }
}

void StandbyState::Reset() {}

// TakenOffState ///////////////////////////////////////////////////////////////
TakenOffState::TakenOffState() {}

void TakenOffState::Handle(::src::controls::Sensors &sensors,
                           ::src::controls::Goal &goal,
                           ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;

  if (goal.run_mission()) {
    output.set_state(ARMED);
  }
}

void TakenOffState::Reset() {}

// TakingOffState //////////////////////////////////////////////////////////////
TakingOffState::TakingOffState() {}

void TakingOffState::Handle(::src::controls::Sensors &sensors,
                            ::src::controls::Goal &goal,
                            ::src::controls::Output &output) {
  (void)goal;

  // Arm the drone before performing a takeoff.
  if (!sensors.armed()) {
    output.set_state(ARMING);
    return;
  }

  // Ensure that the drone reaches a safe altitude before going into the next
  // state.
  if (sensors.relative_altitude() > kTakeoffAltitude) {
    output.set_state(TAKEN_OFF);
  }
}

void TakingOffState::Reset() {}

// UnknownState ////////////////////////////////////////////////////////////////
UnknownState::UnknownState() {}

void UnknownState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;

  // LOG_LINE("Unknown state!");
}

void UnknownState::Reset() {}

} // namespace state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src