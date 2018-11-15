#include "flight_loop.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>

#include "zmq.hpp"

namespace src {
namespace control {
namespace loops {
namespace {
int kFlightLoopFrequency = 1e2;
int kMaxMessageInQueues = 5;
}

FlightLoop::FlightLoop()
    : state_(STANDBY),
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
      sensors_receiver_("ipc:///tmp/uasatucla_sensors.ipc",
                        kMaxMessageInQueues),
      goal_receiver_("ipc:///tmp/uasatucla_goal.ipc", kMaxMessageInQueues),
      status_sender_("ipc:///tmp/uasatucla_status.ipc"),
      output_sender_("ipc:///tmp/uasatucla_output.ipc") {}

void FlightLoop::Iterate() { RunIteration(); }

void FlightLoop::SetVerbose(bool verbose) { verbose_ = verbose; }

void FlightLoop::RunIteration() {
  phased_loop_.SleepUntilNext();

  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  // Get latest telemetry.
  if (!sensors_receiver_.HasMessages()) {
    return;
  }

  // Convert UasMessage to sensor
  ::src::control::UasMessage message = sensors_receiver_.GetLatest();
  ::src::control::Sensors sensors = message.sensors();

  // Get latest goal.
  if (!goal_receiver_.HasMessages()) {
    return;
  }

  ::src::control::Goal goal = goal_receiver_.GetLatest();

  got_sensors_ = true;
  State next_state = state_;

  LOG_LINE("Flight Loop dt: " << std::setprecision(14)
                              << current_time - last_loop_ - 0.01);

  if (current_time - last_loop_ > 0.01 + 0.002) {
    LOG_LINE("Flight LOOP RUNNING SLOW: dt: "
             << std::setprecision(14) << current_time - last_loop_ - 0.01);
  }
  last_loop_ = current_time;

  ::src::control::Output output = ::src::control::Output();

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

  if (goal.trigger_alarm() + 0.05 > current_time) {
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
      if (sensors.last_gps() < current_time - 0.5) {
        LOG_LINE("can't arm; no GPS "
                 << "(last gps: " << sensors.last_gps()
                 << " current time: " << current_time);

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

      pilot::PilotOutput pilot_output = pilot_.Calculate(position, velocity);

      last_bomb_drop_ = pilot_output.bomb_drop ? current_time : last_bomb_drop_;

      if (pilot_output.alarm) {
        alarm_.AddAlert({5.0, 0.50});
      }

      output.set_velocity_x(pilot_output.flight_velocities.x);
      output.set_velocity_y(pilot_output.flight_velocities.y);
      output.set_velocity_z(pilot_output.flight_velocities.z);
      output.set_yaw_setpoint(pilot_output.yaw);
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
  if (next_state == IN_AIR && sensors.last_gps() < current_time - 0.5) {
    LOG_LINE("no GPS; landing (last gps: "
             << sensors.last_gps() << " current time: " << current_time);

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
  if (last_bomb_drop_ <= current_time && last_bomb_drop_ + 5.0 > current_time) {
    output.set_bomb_drop(true);
  }

  // Handle dslr.
  output.set_dslr(false);
  last_dslr_ = ::std::max(last_dslr_, goal.trigger_dslr());
  if (last_dslr_ <= current_time && last_dslr_ + 15.0 > current_time) {
    output.set_dslr(true);
  }

  LOG_LINE("Flight loop iteration OUTPUT..."
           << " VelocityX: " << output.velocity_x() << " VelocityY: "
           << output.velocity_y() << " VelocityZ: " << output.velocity_z());

  // TODO(comran): Send output.

  ::src::control::Status status = ::src::control::Status();
  status.set_state(next_state);
  status.set_current_command_index(0);

  if (current_flight_start_time_ == 0) {
    status.set_flight_time(previous_flights_time_);
  } else {
    status.set_flight_time(
        previous_flights_time_ +
        (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
             .count() -
         current_flight_start_time_));
  }

  LOG_LINE("Flight loop iteration STATUS... "
           << " State: " << status.state()
           << " FlightTime: " << status.flight_time()
           << " CurrentCommandIndex: " << status.current_command_index());

  output_sender_.Send(output);
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

void FlightLoop::Run() {
  running_ = true;

  while (running_) {
    RunIteration();
  }
}

} // namespace loops
} // namespace control
} // namespace src
