#include "mission_state_machine.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace mission_state_machine {

MissionStateMachine::MissionStateMachine() :
    state_(GET_NEXT_CMD),
    unknown_state_(new UnknownState()),
    drone_program_index_(0),
    setpoint_initialized_(false),
    new_mission_ready_(false) {

  // Create an instance of all state handlers.
  state_handlers_[TRANSLATE] = new TranslateState();
  state_handlers_[UGV_DROP] = new UGVDropState();
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

  // Set initial setpoint.
  if (!setpoint_initialized_) {
    setpoint_latitude_ = sensors.latitude();
    setpoint_longitude_ = sensors.longitude();
    setpoint_altitude_ = sensors.relative_altitude();
    setpoint_yaw_ = 90;
    setpoint_initialized_ = true;
  }

  output.set_setpoint_latitude(setpoint_latitude_);
  output.set_setpoint_longitude(setpoint_longitude_);
  output.set_setpoint_altitude(setpoint_altitude_);
  output.set_setpoint_yaw(setpoint_yaw_);

  // If a new mission is available, escape any current state and go to the new
  // mission.
  if (new_mission_ready_) {
    state_ = GET_NEXT_CMD;
    new_mission_ready_ = false;
  }

  // Use same state in next loop iteration, unless it is changed.
  output.set_mission_state(state_);

  // Handle current state.
  if (state_ != GET_NEXT_CMD) {
    GetStateHandler(state_)->Handle(sensors, goal, output);
  }

  // Transition to next state.
  StateTransition(output);

  // Keep track of any changes to setpoint.
  setpoint_latitude_ = output.setpoint_latitude();
  setpoint_longitude_ = output.setpoint_longitude();
  setpoint_altitude_ = output.setpoint_altitude();
  setpoint_yaw_ = output.setpoint_yaw();

  output.set_trigger_offboard(true);
  output.set_send_setpoint(true);
}

void MissionStateMachine::StateTransition(::src::controls::Output &output) {
  MissionState old_state = state_;
  MissionState new_state = static_cast<MissionState>(output.mission_state());

  // Do nothing if state does not change.
  if (old_state == new_state && new_state != GET_NEXT_CMD) {
    return;
  }

  // Filter out states that can be transitioned to.
  if (new_state != GET_NEXT_CMD) {
    ROS_ERROR("Cannot transition to any state except GET_NEXT_CMD!");
    return;
  }

  // Load next command.
  {
    ::std::lock_guard<::std::mutex> lock(drone_program_mutex_);

    if (drone_program_.commands_size() > drone_program_index_) {
      ground_controls::timeline::DroneCommand loaded_command =
          drone_program_.commands(drone_program_index_);
      ROS_DEBUG_STREAM("COMMAND: " << loaded_command.has_translate_command());
      if (loaded_command.has_translate_command()) {
        new_state = TRANSLATE;
        ((TranslateState *)GetStateHandler(TRANSLATE))
            ->SetSetpoints(
                loaded_command.translate_command().goal().latitude(),
                loaded_command.translate_command().goal().longitude(),
                loaded_command.translate_command().goal().altitude(), -90);
      } else if (loaded_command.has_trigger_bomb_drop_command()) {
        new_state = UGV_DROP;
      }

      drone_program_index_++;
    } else {
      // Empty drone program, so try to load the next available command until
      // one is available.
      new_state = GET_NEXT_CMD;
    }
  }

  // Log all state transitions that aren't GET_NEXT_CMD.
  if (new_state != GET_NEXT_CMD) {
    ROS_INFO("Switching states: %s -> %s", StateToString(old_state).c_str(),
             StateToString(new_state).c_str());
  }

  if (new_state != old_state) {
    // Apply the state transition.
    state_ = new_state;

    // Reset the new state after transition is made.
    GetStateHandler(state_)->Reset();
  }
}

State *MissionStateMachine::GetStateHandler(MissionState state) {
  if (state_handlers_.count(state)) {
    return state_handlers_[state];
  }

  return unknown_state_;
}

void MissionStateMachine::LoadMission(
    ::src::controls::ground_controls::timeline::DroneProgram drone_program) {

  ::std::lock_guard<::std::mutex> lock(drone_program_mutex_);
  drone_program_ = drone_program;
  drone_program_index_ = 0;
  new_mission_ready_ = true;
}

::std::string StateToString(MissionState state) {
  switch (state) {
    case TRANSLATE:
      return "TRANSLATE";
    case UGV_DROP:
      return "UGV_DROP";
    case GET_NEXT_CMD:
      return "GET_NEXT_CMD";
  }
  return "UNKNOWN";
}

// TranslateState //////////////////////////////////////////////////////////////
TranslateState::TranslateState() {}

void TranslateState::Handle(::src::controls::Sensors &sensors,
                            ::src::controls::Goal &goal,
                            ::src::controls::Output &output) {
  (void)goal;

  // Write setpoint outputs.
  output.set_send_setpoint(true);
  output.set_setpoint_latitude(setpoint_latitude_);
  output.set_setpoint_longitude(setpoint_longitude_);
  output.set_setpoint_altitude(setpoint_altitude_);
  output.set_setpoint_yaw(setpoint_yaw_);

  // Check if acceptance radius has been met.
  ::lib::Position3D drone, destination;
  drone.latitude = sensors.latitude();
  drone.longitude = sensors.longitude();
  drone.altitude = sensors.relative_altitude();
  destination.latitude = setpoint_latitude_;
  destination.longitude = setpoint_longitude_;
  destination.altitude = setpoint_altitude_;

  double distance_from_destination = GetDistance3D(drone, destination);
  // ROS_DEBUG("Distance from dest: %f", distance_from_destination);

  if (distance_from_destination < kAcceptanceRadius) {
    output.set_mission_state(GET_NEXT_CMD);
  }
}

void TranslateState::Reset() {}

void TranslateState::SetSetpoints(double latitude, double longitude,
                                  double altitude, double yaw) {
  setpoint_latitude_ = latitude;
  setpoint_longitude_ = longitude;
  setpoint_altitude_ = altitude;
  setpoint_yaw_ = yaw;
}

// UGVDropState ////////////////////////////////////////////////////////////////
UGVDropState::UGVDropState() {}

void UGVDropState::Handle(::src::controls::Sensors &sensors,
                          ::src::controls::Goal &goal,
                          ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;

  output.set_deploy(true);
}

void UGVDropState::Reset() {}

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
