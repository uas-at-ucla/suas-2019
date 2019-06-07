#include "flight_state_machine.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace flight_state_machine {

FlightStateMachine::FlightStateMachine() :
    state_(STANDBY),
    unknown_state_(new UnknownState()) {

  // Create an instance of all state handlers.
  state_handlers_[STANDBY] = new StandbyState();
  state_handlers_[ARMING] = new ArmingState();
  state_handlers_[ARMED_WAIT_FOR_SPINUP] = new ArmedWaitForSpinupState();
  state_handlers_[TAKING_OFF] = new TakingOffState();
  state_handlers_[TAKEN_OFF] = new TakenOffState();
  state_handlers_[SAFETY_PILOT_CONTROL] = new SafetyPilotControlState();
  state_handlers_[MISSION] = new MissionState();
}

FlightStateMachine::~FlightStateMachine() {
  // Delete all state handler instances.
  for (auto const &state_handler_pair : state_handlers_) {
    delete state_handler_pair.second;
  }

  delete unknown_state_;
}

void FlightStateMachine::Handle(::src::controls::Sensors &sensors,
                                ::src::controls::Goal &goal,
                                ::src::controls::Output &output) {

  // Use same state in next loop iteration, unless it is changed.
  output.set_state(state_);

  // Handle current state.
  GetStateHandler(state_)->Handle(sensors, goal, output);

  // Transition to next state.
  StateTransition(output);
}

void FlightStateMachine::LoadMission(
    ::src::controls::ground_controls::timeline::DroneProgram drone_program) {
  ((MissionState *)GetStateHandler(MISSION))->LoadMission(drone_program);
}

void FlightStateMachine::StateTransition(::src::controls::Output &output) {
  // Record the old and new states.
  FlightLoopState old_state = state_;
  FlightLoopState new_state = static_cast<FlightLoopState>(output.state());

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

State *FlightStateMachine::GetStateHandler(FlightLoopState state) {
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
  }

  return "UNKNOWN";
}

// StandbyState ////////////////////////////////////////////////////////////////
StandbyState::StandbyState() {}

void StandbyState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  (void)sensors;

  // Do nothing if a mission is currently not active.
  if (!goal.run_mission()) {
    return;
  }

  if (sensors.armed()) {
    ROS_INFO("Pixhawk is already armed; switching to ArmedWaitForSpinupState "
             "state.");
    output.set_state(ARMED_WAIT_FOR_SPINUP);
    return;
  }

  ROS_INFO("Run mission requested and not armed; attempting to arm.");
  output.set_state(ARMING);
}

void StandbyState::Reset() {}

// ArmingState /////////////////////////////////////////////////////////////////
ArmingState::ArmingState() : start_(0), was_reset_(true) {}

void ArmingState::Handle(::src::controls::Sensors &sensors,
                         ::src::controls::Goal &goal,
                         ::src::controls::Output &output) {
  (void)goal;

  // TODO(comran): Check that throttle is at zero before arming.
  // TODO(comran): Check if we have GPS.

  if (!goal.run_mission()) {
    output.set_state(STANDBY);
    return;
  }

  if (sensors.armed() && sensors.relative_altitude() > kMinTakeoffAltitude) {
    output.set_state(TAKEN_OFF);
    return;
  }

  if (sensors.armed()) {
    output.set_state(ARMED_WAIT_FOR_SPINUP);
    return;
  }

  if (was_reset_) {
    start_ = sensors.time();
    was_reset_ = false;
  }

  bool arm_trigger = ::std::fmod(sensors.time() - start_, kTriggerPeriod) >
                     kTriggerPeriod / 2.0;
  output.set_trigger_arm(arm_trigger);
  output.set_trigger_hold(true);
}

void ArmingState::Reset() {
  start_ = 0;
  was_reset_ = true;
}

// ArmedWaitForSpinupState /////////////////////////////////////////////////////
ArmedWaitForSpinupState::ArmedWaitForSpinupState() :
    start_(0),
    was_reset_(true) {}

void ArmedWaitForSpinupState::Handle(::src::controls::Sensors &sensors,
                                     ::src::controls::Goal &goal,
                                     ::src::controls::Output &output) {
  if (!goal.run_mission()) {
    output.set_state(LANDING);
    return;
  }

  if (!sensors.armed()) {
    output.set_state(ARMING);
    return;
  }

  if (sensors.armed() && sensors.relative_altitude() > kMinTakeoffAltitude) {
    output.set_state(TAKEN_OFF);
    return;
  }

  // Set initial time if it was not set yet.
  if (was_reset_) {
    start_ = sensors.time();
    was_reset_ = false;
  }

  // Wait a bit for the propellers to spin up while armed.
  if (sensors.time() - start_ > kSpinupTime) {
    output.set_state(TAKING_OFF);
  }
}

void ArmedWaitForSpinupState::Reset() {
  start_ = 0;
  was_reset_ = true;
}

// TakingOffState //////////////////////////////////////////////////////////////
TakingOffState::TakingOffState() {}

void TakingOffState::Handle(::src::controls::Sensors &sensors,
                            ::src::controls::Goal &goal,
                            ::src::controls::Output &output) {
  if (!goal.run_mission()) {
    output.set_state(LANDING);
    return;
  }

  // Arm the drone before performing a takeoff.
  if (!sensors.armed()) {
    output.set_state(ARMING);
    return;
  }

  // Ensure that the drone reaches a safe altitude before going into the next
  // state.
  if (sensors.autopilot_state() == kPixhawkCustomModeLoiter &&
      sensors.relative_altitude() > kMinTakeoffAltitude) {
    output.set_state(TAKEN_OFF);
    return;
  }

  // Send one constant trigger signal. Sending multiple would cause the drone
  // to keep increasing in altitude by the takeoff height.
  output.set_trigger_takeoff(true);
}

void TakingOffState::Reset() {}

// TakenOffState ///////////////////////////////////////////////////////////////
TakenOffState::TakenOffState() : start_(0), was_reset_(true) {}

void TakenOffState::Handle(::src::controls::Sensors &sensors,
                           ::src::controls::Goal &goal,
                           ::src::controls::Output &output) {

  // If we no longer want to run a mission, attempt to land.
  if (!goal.run_mission()) {
    output.set_state(LANDING);
    return;
  }

  // If we somehow got disarmed after taking off (not sure how that'd happen),
  // try to go through the arming proceedure again.
  if (!sensors.armed()) {
    output.set_state(ARMING);
    return;
  }

  // Take off again if we drop below minimum altitude.
  if (sensors.relative_altitude() < kMinTakeoffAltitude) {
    output.set_state(TAKING_OFF);
    return;
  }

  // Set initial time if it was not set yet.
  if (was_reset_) {
    start_ = sensors.time();
    was_reset_ = false;
  }

  // Wait a bit in taken off state before continuing to mission.
  if (sensors.time() - start_ > kTakenOffWaitTime) {
    output.set_state(MISSION);
  }
}

void TakenOffState::Reset() {
  start_ = 0;
  was_reset_ = true;
}

// MissionState ////////////////////////////////////////////////////////////////
MissionState::MissionState() {}

void MissionState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {

  (void)sensors;

  if (!goal.run_mission()) {
    output.set_state(SAFETY_PILOT_CONTROL);
    return;
  }

  mission_state_machine_.Handle(sensors, goal, output);
}

void MissionState::Reset() {}

void MissionState::LoadMission(
    ::src::controls::ground_controls::timeline::DroneProgram drone_program) {
  mission_state_machine_.LoadMission(drone_program);
}

// SafetyPilotControlState /////////////////////////////////////////////////////
SafetyPilotControlState::SafetyPilotControlState() {}

void SafetyPilotControlState::Handle(::src::controls::Sensors &sensors,
                                     ::src::controls::Goal &goal,
                                     ::src::controls::Output &output) {
  // If UAS mission mode is requested again, return to that state if altitude
  // is above takeoff threshold.
  if (goal.run_mission() &&
      sensors.relative_altitude() >= kMinTakeoffAltitude) {
    output.set_state(MISSION);
    return;
  }

  // If drone was manually disarmed, go back to standby.
  if (!sensors.armed()) {
    output.set_state(STANDBY);
    return;
  }
}

void SafetyPilotControlState::Reset() {}

// LandingState ////////////////////////////////////////////////////////////////
LandingState::LandingState() {}

void LandingState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  (void)goal;

  if (!sensors.armed()) {
    output.set_state(STANDBY);
    return;
  }

  output.set_trigger_land(true);
}

void LandingState::Reset() {}

// UnknownState ////////////////////////////////////////////////////////////////
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

} // namespace flight_state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src
