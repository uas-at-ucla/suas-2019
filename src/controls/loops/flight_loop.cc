#include "flight_loop.h"

namespace src {
namespace controls {
namespace loops {

::std::string StateToString(State state) {
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
    case AUTOPILOT:
      return "AUTOPILOT";
    case LANDING:
      return "LANDING";
    case FAILSAFE:
      return "FAILSAFE";
    case FLIGHT_TERMINATION:
      return "FLIGHT_TERMINATION";
  }
  return "UNKNOWN";
}

FlightLoop::FlightLoop() :
    state_(STANDBY),
    running_(false),
    phased_loop_(kFlightLoopFrequency),
    start_(std::chrono::system_clock::now()),
    takeoff_ticker_(0),
    previous_flights_time_(0),
    current_flight_start_time_(0),
    alarm_(kFlightLoopFrequency),
    last_loop_(0),
    did_alarm_(false),
    did_arm_(false),
    last_bomb_drop_(0),
    last_dslr_(0),
    sensors_receiver_("ipc:///tmp/uasatucla_sensors.ipc", kMaxMessageInQueues),
    goal_receiver_("ipc:///tmp/uasatucla_goal.ipc", kMaxMessageInQueues),
    output_sender_("ipc:///tmp/uasatucla_output.ipc") {}

void FlightLoop::Run() {
  sensors_receiver_.Connect();
  goal_receiver_.Connect();
  output_sender_.Connect();

  // Don't allow two instances of the loop to run simultaneously on different
  // threads.
  if (running_) {
    LOG_LINE("Loop already running.");
    return;
  }
  running_ = true;

  while (running_) {
    // Run loop at a set frequency.
    phased_loop_.SleepUntilNext();

    // Fetch latest messages.
    if (!sensors_receiver_.HasMessages()) {
      continue;
    }

    if (!goal_receiver_.HasMessages()) {
      continue;
    }

    // Extract messages from UasMessage wrapper.
    ::src::controls::UasMessage sensors_uas_message =
        sensors_receiver_.GetLatest();
    if (!sensors_uas_message.has_sensors()) {
      continue;
    }
    ::src::controls::Sensors sensors_message = sensors_uas_message.sensors();

    ::src::controls::UasMessage goal_uas_message = goal_receiver_.GetLatest();
    if (!goal_uas_message.has_goal()) {
      continue;
    }
    ::src::controls::Goal goal_message = goal_uas_message.goal();

    // Set the current time in inputted sensors message.
    double current_time = ::lib::phased_loop::GetCurrentTime();
    sensors_message.set_time(current_time);

    // Run control loop iteration.
    ::src::controls::Output output_message =
        RunIteration(sensors_message, goal_message);

    // Send out output message.
    ::src::controls::UasMessage output_uas_message;
    output_uas_message.set_allocated_output(&output_message);
    output_sender_.Send(output_uas_message);
  }
}

::src::controls::Output
FlightLoop::RunIteration(::src::controls::Sensors sensors,
                         ::src::controls::Goal goal) {
  LOG_LINE("Running flight loop iteration @ "
           << ::std::fixed << ::std::setw(8) << ::std::setprecision(3)
           << sensors.time() << " with state: " << StateToString(state_));

  ::src::controls::Output output = GenerateDefaultOutput();

  if (SafetyStateOverride(goal, output)) {
    StateTransition(output);
  }

  RouteToCurrentState(sensors, goal, output);
  StateTransition(output);

  WriteActuators(sensors, goal, output);

  if (current_flight_start_time_ == 0) {
    output.set_flight_time(previous_flights_time_);
  } else {
    output.set_flight_time(
        previous_flights_time_ +
        (::lib::phased_loop::GetCurrentTime() - current_flight_start_time_));
  }

  return output;
}

bool FlightLoop::SafetyStateOverride(::src::controls::Goal &goal,
                                     ::src::controls::Output &output) {
  // Prioritize the throttle cut check, so that the drone always cuts throttle
  // instead of following any failsafe commands.
  if (goal.trigger_throttle_cut()) {
    EndFlightTimer();
    output.set_state(FLIGHT_TERMINATION);
    return true;
  }

  if (goal.trigger_failsafe()) {
    EndFlightTimer();
    output.set_state(FAILSAFE);
    return true;
  }

  return false;
}

void FlightLoop::MonitorLoopFrequency(::src::controls::Sensors sensors) {
  LOG_LINE("Flight Loop dt: " << ::std::setprecision(14)
                              << sensors.time() - last_loop_ - 0.01);

  if (sensors.time() - last_loop_ > 0.01 + 0.002) {
    LOG_LINE("Flight LOOP RUNNING SLOW: dt: "
             << std::setprecision(14) << sensors.time() - last_loop_ - 0.01);
  }
  last_loop_ = sensors.time();
}

void FlightLoop::EndFlightTimer() {
  if (current_flight_start_time_ != 0) {
    previous_flights_time_ +=
        ::lib::phased_loop::GetCurrentTime() - current_flight_start_time_;
    current_flight_start_time_ = 0;
  }
}

::src::controls::Output FlightLoop::GenerateDefaultOutput() {
  ::src::controls::Output output;

  // Set state to integer representation of the current state of the flight
  // loop.
  output.set_state(static_cast<int>(state_));
  output.set_flight_time(0);
  output.set_current_command_index(0);

  output.set_velocity_x(0);
  output.set_velocity_y(0);
  output.set_velocity_z(0);
  output.set_yaw_setpoint(0);

  output.set_gimbal_angle(kDefaultGimbalAngle);
  output.set_bomb_drop(false);
  output.set_alarm(false);
  output.set_dslr(false);

  output.set_trigger_takeoff(0);
  output.set_trigger_hold(0);
  output.set_trigger_offboard(0);
  output.set_trigger_rtl(0);
  output.set_trigger_land(0);
  output.set_trigger_arm(0);
  output.set_trigger_disarm(0);

  return output;
}

void FlightLoop::StateTransition(::src::controls::Output &output) {
  State old_state = state_;
  State new_state = static_cast<State>(output.state());

  if (old_state != new_state) {
    // Handle state transitions.
    LOG_LINE("Switching states: " << StateToString(old_state) << " -> "
                                  << StateToString(new_state));
  }

  state_ = new_state;
}

void FlightLoop::WriteActuators(::src::controls::Sensors &sensors,
                                ::src::controls::Goal &goal,
                                ::src::controls::Output &output) {
  // Handle alarm.
  output.set_alarm(alarm_.ShouldAlarm());

  if (goal.trigger_alarm() + 0.05 > sensors.time()) {
    if (!did_alarm_) {
      did_alarm_ = true;
      alarm_.AddAlert({0.30, 0.30});
      LOG_LINE("Alarm was manually triggered");
    }
  } else {
    did_alarm_ = false;
  }

  if (sensors.armed() && !did_arm_) {
    // Send out a chirp if the Pixhawk just got armed.
    did_arm_ = true;
    alarm_.AddAlert({0.03, 0.25});
  }

  if (!sensors.armed()) {
    did_arm_ = false;
  }

  // Handle bomb drop.
  last_bomb_drop_ = ::std::max(last_bomb_drop_, goal.trigger_bomb_drop());

  output.set_bomb_drop(false);
  if (last_bomb_drop_ <= sensors.time() &&
      last_bomb_drop_ + 5.0 > sensors.time()) {
    output.set_bomb_drop(true);
  }

  // Handle dslr.
  output.set_dslr(false);
  last_dslr_ = ::std::max(last_dslr_, goal.trigger_dslr());
  if (last_dslr_ <= sensors.time() && last_dslr_ + 15.0 > sensors.time()) {
    output.set_dslr(true);
  }
}

// State machine handlers //////////////////////////////////////////////////////
void FlightLoop::RouteToCurrentState(::src::controls::Sensors &sensors,
                                     ::src::controls::Goal &goal,
                                     ::src::controls::Output &output) {
  switch (state_) {
    case STANDBY:
      HandleStandby(sensors, goal, output);
      break;

    case ARMING:
      HandleArming(sensors, goal, output);
      break;

    case ARMED_WAIT_FOR_SPINUP:
      HandleArmedWaitForSpinup(sensors, goal, output);
      break;

    case ARMED:
      HandleArmed(sensors, goal, output);
      break;

    case TAKING_OFF:
      HandleTakingOff(sensors, goal, output);
      break;

    case TAKEN_OFF:
      HandleTakenOff(sensors, goal, output);
      break;

    case SAFETY_PILOT_CONTROL:
      HandleSafetyPilotControl(sensors, goal, output);
      break;

    case AUTOPILOT: {
      HandleAutopilot(sensors, goal, output);
      break;
    }

    case LANDING:
      HandleLanding(sensors, goal, output);
      break;

    case FAILSAFE:
      HandleFailsafe(sensors, goal, output);
      break;

    case FLIGHT_TERMINATION:
      HandleFlightTermination(sensors, goal, output);
      break;
  }
}

void FlightLoop::HandleStandby(::src::controls::Sensors &sensors,
                               ::src::controls::Goal &goal,
                               ::src::controls::Output &output) {
  if (goal.run_mission()) {
    LOG_LINE("Run mission requested; attempting to arm.");
    output.set_state(ARMING);
  }

  if (sensors.armed()) {
    LOG_LINE("Pixhawk was armed; switching to ARMED state.");
    output.set_state(ARMED);
  }
}

void FlightLoop::HandleArming(::src::controls::Sensors &sensors,
                              ::src::controls::Goal &goal,
                              ::src::controls::Output &output) {
  (void) goal;
  // Check if we have GPS.
  if (sensors.last_gps() < sensors.time() - 0.5) {
    LOG_LINE("can't arm; no GPS "
             << "(last gps: " << sensors.last_gps()
             << " current time: " << sensors.time());

    output.set_state(STANDBY);
  }

  if (sensors.armed()) {
    output.set_state(ARMED);
  }

  output.set_trigger_arm(sensors.time());
}

void FlightLoop::HandleArmedWaitForSpinup(::src::controls::Sensors &sensors,
                                          ::src::controls::Goal &goal,
                                          ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}
void FlightLoop::HandleArmed(::src::controls::Sensors &sensors,
                             ::src::controls::Goal &goal,
                             ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;

  current_flight_start_time_ = ::lib::phased_loop::GetCurrentTime();

  if (!sensors.armed()) {
    if (goal.run_mission()) {
      output.set_state(ARMING);
    } else {
      output.set_state(STANDBY);
    }
  }
}
void FlightLoop::HandleTakingOff(::src::controls::Sensors &sensors,
                                 ::src::controls::Goal &goal,
                                 ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
  if (!goal.run_mission()) {
    takeoff_ticker_ = 0;
    output.set_state(LANDING);
    return;
  }

  if (!sensors.armed()) {
    takeoff_ticker_ = 0;
    output.set_state(ARMING);
    return;
  }

  if (sensors.relative_altitude() < 0.3) {
    takeoff_ticker_++;
  }

  if (sensors.relative_altitude() > 2.2) {
    takeoff_ticker_ = 0;
    output.set_state(AUTOPILOT);
  }
}
void FlightLoop::HandleTakenOff(::src::controls::Sensors &sensors,
                                ::src::controls::Goal &goal,
                                ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}
void FlightLoop::HandleSafetyPilotControl(src::controls::Sensors &sensors,
                                          ::src::controls::Goal &goal,
                                          ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}
void FlightLoop::HandleAutopilot(::src::controls::Sensors &sensors,
                                 ::src::controls::Goal &goal,
                                 ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
  Position3D position = {sensors.latitude(), sensors.longitude(),
                         sensors.relative_altitude()};

  ::Eigen::Vector3d velocity(sensors.velocity_x(), sensors.velocity_y(),
                             sensors.velocity_z());

  executor::ExecutorOutput executor_output =
      executor_.Calculate(position, velocity);

  last_bomb_drop_ =
      executor_output.bomb_drop ? sensors.time() : last_bomb_drop_;

  if (executor_output.alarm) {
    alarm_.AddAlert({5.0, 0.50});
  }

  output.set_velocity_x(executor_output.flight_velocities.x);
  output.set_velocity_y(executor_output.flight_velocities.y);
  output.set_velocity_z(executor_output.flight_velocities.z);
  output.set_yaw_setpoint(executor_output.yaw);
}
void FlightLoop::HandleLanding(::src::controls::Sensors &sensors,
                               ::src::controls::Goal &goal,
                               ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
  if (!sensors.armed()) {
    EndFlightTimer();
    output.set_state(STANDBY);
    return;
  }

  if (goal.run_mission() && sensors.relative_altitude() > 5.0) {
    output.set_state(AUTOPILOT);
  }
}
void FlightLoop::HandleFailsafe(::src::controls::Sensors &sensors,
                                ::src::controls::Goal &goal,
                                ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}
void FlightLoop::HandleFlightTermination(::src::controls::Sensors &sensors,
                                         ::src::controls::Goal &goal,
                                         ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
}

} // namespace loops
} // namespace controls
} // namespace src
