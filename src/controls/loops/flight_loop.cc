#include "flight_loop.h"

namespace src {
namespace controls {
namespace loops {

FlightLoop::FlightLoop() :
    state_(STANDBY),
    running_(false),
    phased_loop_(kFlightLoopFrequency),
    start_(std::chrono::system_clock::now()),
    takeoff_ticker_(0),
    verbose_(false),
    previous_flights_time_(0),
    current_flight_start_time_(0),
    alarm_(kFlightLoopFrequency),
    got_sensors_(false),
    last_loop_(0),
    did_alarm_(false),
    did_arm_(false),
    last_bomb_drop_(0),
    last_dslr_(0),
    sensors_receiver_("ipc:///tmp/uasatucla_sensors.ipc", kMaxMessageInQueues),
    goal_receiver_("ipc:///tmp/uasatucla_goal.ipc", kMaxMessageInQueues),
    output_sender_("ipc:///tmp/uasatucla_output.ipc") {}

void FlightLoop::SetVerbose(bool verbose) { verbose_ = verbose; }

void FlightLoop::Run() {
  // Don't allow two instances of the loop to run simultaneously on different
  // threads.
  if(running_) {
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
    double current_time =
    ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
        ::std::chrono::system_clock::now().time_since_epoch())
        .count() *
    1e-9;
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
  got_sensors_ = true;
  State next_state = state_;

  LOG_LINE("Flight Loop dt: " << std::setprecision(14)
                              << sensors.time() - last_loop_ - 0.01);

  if (sensors.time() - last_loop_ > 0.01 + 0.002) {
    LOG_LINE("Flight LOOP RUNNING SLOW: dt: "
             << std::setprecision(14) << sensors.time() - last_loop_ - 0.01);
  }
  last_loop_ = sensors.time();

  ::src::controls::Output output = ::src::controls::Output();

  output.set_gimbal_angle(0.15);

  output.set_trigger_takeoff(goal.trigger_takeoff());
  output.set_trigger_hold(goal.trigger_hold());
  output.set_trigger_offboard(goal.trigger_offboard());
  output.set_trigger_rtl(goal.trigger_rtl());
  output.set_trigger_land(goal.trigger_land());
  output.set_trigger_arm(goal.trigger_arm());
  output.set_trigger_disarm(goal.trigger_disarm());

  if (goal.trigger_failsafe()) {
    EndFlightTimer();
    next_state = FAILSAFE;
  }

  if (goal.trigger_throttle_cut()) {
    EndFlightTimer();
    next_state = FLIGHT_TERMINATION;
  }

  if (goal.trigger_alarm() + 0.05 > sensors.time()) {
    if (!did_alarm_) {
      did_alarm_ = true;
      alarm_.AddAlert({0.30, 0.30});
      LOG_LINE("Alarm was manually triggered");
    }
  } else {
    did_alarm_ = false;
  }

  // Check if the Pixhawk just got armed, and send out a chirp if it did.
  if (sensors.armed() && !did_arm_) {
    did_arm_ = true;

    alarm_.AddAlert({0.03, 0.25});
  }

  if (!sensors.armed()) {
    did_arm_ = false;
  }

  // Set defaults for all outputs.
  output.set_velocity_x(0);
  output.set_velocity_y(0);
  output.set_velocity_z(0);
  output.set_yaw_setpoint(0);
  output.set_alarm(false);

  bool run_mission = goal.run_mission();

  switch (state_) {
    case STANDBY:
      if (run_mission) {
        next_state = ARMING;
      }

      break;

    case ARMING:
      // Check if we have GPS.
      if (sensors.last_gps() < sensors.time() - 0.5) {
        LOG_LINE("can't arm; no GPS "
                 << "(last gps: " << sensors.last_gps()
                 << " current time: " << sensors.time());

        next_state = STANDBY;
        break;
      }

      if (!run_mission) {
        next_state = LANDING;
        break;
      }

      if (sensors.armed()) {
        next_state = ARMED;
        break;
      }
      break;

    case ARMED:
      if (!run_mission) {
        next_state = LANDING;
        break;
      }

      if (!sensors.armed()) {
        next_state = ARMING;
        break;
      }

      current_flight_start_time_ =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count();
      next_state = TAKING_OFF;
      break;

    case TAKING_OFF:
      if (!run_mission) {
        takeoff_ticker_ = 0;
        next_state = LANDING;
        break;
      }

      if (!sensors.armed()) {
        takeoff_ticker_ = 0;
        next_state = ARMING;
        break;
      }

      if (sensors.relative_altitude() < 0.3) {
        takeoff_ticker_++;
      }

      if (sensors.relative_altitude() > 2.2) {
        takeoff_ticker_ = 0;
        next_state = IN_AIR;
      }
      break;

    case IN_AIR: {
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
      break;
    }

    case LANDING:
      if (!sensors.armed()) {
        EndFlightTimer();
        next_state = STANDBY;
        break;
      }

      if (goal.run_mission() && sensors.relative_altitude() > 5.0) {
        next_state = IN_AIR;
        break;
      }

      break;

    case FAILSAFE:
      break;

    case FLIGHT_TERMINATION:
      break;
  }

  // Land if the GPS data is old.
  if (next_state == IN_AIR && sensors.last_gps() < sensors.time() - 0.5) {
    LOG_LINE("no GPS; landing (last gps: "
             << sensors.last_gps() << " current time: " << sensors.time());

    next_state = LANDING;
  }

  if (next_state != state_) {
    // Handle state transitions.
    LOG_LINE("Switching states: " << state_ << " -> " << next_state);
  }

  state_ = next_state;

  output.set_alarm(alarm_.ShouldAlarm());

  // Handle bomb drop.
  last_bomb_drop_ = ::std::max(last_bomb_drop_, goal.trigger_bomb_drop());

  output.set_bomb_drop(false);
  if (last_bomb_drop_ <= sensors.time() && last_bomb_drop_ + 5.0 > sensors.time()) {
    output.set_bomb_drop(true);
  }

  // Handle dslr.
  output.set_dslr(false);
  last_dslr_ = ::std::max(last_dslr_, goal.trigger_dslr());
  if (last_dslr_ <= sensors.time() && last_dslr_ + 15.0 > sensors.time()) {
    output.set_dslr(true);
  }

  LOG_LINE("Flight loop iteration OUTPUT..."
           << " VelocityX: " << output.velocity_x() << " VelocityY: "
           << output.velocity_y() << " VelocityZ: " << output.velocity_z());

  // TODO(comran): Send output.

  output.set_state(next_state);
  output.set_current_command_index(0);

  if (current_flight_start_time_ == 0) {
    output.set_flight_time(previous_flights_time_);
  } else {
    output.set_flight_time(
        previous_flights_time_ +
        (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
             .count() -
         current_flight_start_time_));
  }

  return output;
}

void FlightLoop::EndFlightTimer() {
  if (current_flight_start_time_ != 0) {
    previous_flights_time_ +=
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count() -
        current_flight_start_time_;
    current_flight_start_time_ = 0;
  }
}

} // namespace loops
} // namespace controls
} // namespace src
