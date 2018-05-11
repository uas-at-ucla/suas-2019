#include "flight_loop.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>

#include "zmq.hpp"

#include "src/control/loops/flight_loop.q.h"

namespace src {
namespace control {
namespace loops {

int kFlightLoopFrequency = 1e2;

FlightLoop::FlightLoop()
    : state_(STANDBY),
      running_(false),
      phased_loop_(::std::chrono::milliseconds(
                       static_cast<int>(1e3 / kFlightLoopFrequency)),
                   ::std::chrono::milliseconds(0)),
      start_(std::chrono::system_clock::now()),
      takeoff_ticker_(0),
      verbose_(false),
      count_(0),
      previous_flights_time_(0),
      current_flight_start_time_(0),
      alarm_(kFlightLoopFrequency),
      got_sensors_(false),
      last_loop_(0),
      did_alarm_(false),
      did_arm_(false) {
  ::src::control::loops::flight_loop_queue.sensors.FetchLatest();
  ::src::control::loops::flight_loop_queue.goal.FetchLatest();
  ::src::control::loops::flight_loop_queue.output.FetchLatest();
}

void FlightLoop::Iterate() { RunIteration(); }

void FlightLoop::DumpSensorsPeriodic() {
  if (!verbose_) return;

  if (count_++ % 100) return;

  DumpSensors();
}

void FlightLoop::DumpSensors() {
  if (::src::control::loops::flight_loop_queue.sensors.get()) {
    LOG_LINE(
        "Flight loop iteration SENSORS..."
        << " Armed:" << ::src::control::loops::flight_loop_queue.sensors->armed
        << ::std::setprecision(12) << ::std::fixed << ::std::showpos
        << " Latitude: "
        << ::src::control::loops::flight_loop_queue.sensors->latitude
        << " Longitude: "
        << ::src::control::loops::flight_loop_queue.sensors->longitude
        << " Altitude: "
        << ::src::control::loops::flight_loop_queue.sensors->altitude
        << " RelativeAltitude: "
        << ::src::control::loops::flight_loop_queue.sensors->relative_altitude
        << " AccelX: "
        << ::src::control::loops::flight_loop_queue.sensors->accelerometer_x
        << " AccelY: "
        << ::src::control::loops::flight_loop_queue.sensors->accelerometer_y
        << " AccelZ: "
        << ::src::control::loops::flight_loop_queue.sensors->accelerometer_z
        << " GyroX: "
        << ::src::control::loops::flight_loop_queue.sensors->gyro_x
        << " GyroY: "
        << ::src::control::loops::flight_loop_queue.sensors->gyro_y
        << " GyroZ: "
        << ::src::control::loops::flight_loop_queue.sensors->gyro_z
        << " AbsolutePressure: "
        << ::src::control::loops::flight_loop_queue.sensors->absolute_pressure
        << " RelativePressure: "
        << ::src::control::loops::flight_loop_queue.sensors->relative_pressure
        << " PressureAltitude: "
        << ::src::control::loops::flight_loop_queue.sensors->pressure_altitude
        << " Temperature: "
        << ::src::control::loops::flight_loop_queue.sensors->temperature
        << " BattVoltage: "
        << ::src::control::loops::flight_loop_queue.sensors->battery_voltage
        << " BattCurrent: "
        << ::src::control::loops::flight_loop_queue.sensors->battery_current);
  }

  if (::src::control::loops::flight_loop_queue.goal.get()) {
    LOG_LINE(
        "Flight loop iteration GOAL... "
        << " RunMission: "
        << ::src::control::loops::flight_loop_queue.goal->run_mission
        << " Failsafe: "
        << ::src::control::loops::flight_loop_queue.goal->trigger_failsafe
        << " ThrottleCut: "
        << ::src::control::loops::flight_loop_queue.goal->trigger_throttle_cut);
  }
}

void FlightLoop::SetVerbose(bool verbose) { verbose_ = verbose; }

void FlightLoop::RunIteration() {
  // TODO(comran): Are these two queues synced?
  // TODO(comran): Check for stale queue messages.
  ::src::control::loops::flight_loop_queue.sensors.FetchAnother();
  ::src::control::loops::flight_loop_queue.goal.FetchLatest();

  if (!got_sensors_) {
    // Send out an alarm chirp to signal that the drone loop is running
    // successfully.
    alarm_.AddAlert({0.02, 0.15});
    alarm_.AddAlert({0.02, 0.15});
  }

  got_sensors_ = true;

  DumpSensors();

  State next_state = state_;

  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;
  LOG_LINE("Flight Loop dt: " << std::setprecision(14)
                              << current_time - last_loop_ - 0.01);
  if (current_time - last_loop_ > 0.01 + 0.002) {
    LOG_LINE("Flight LOOP RUNNING SLOW: dt: "
             << std::setprecision(14) << current_time - last_loop_ - 0.01);
  }
  last_loop_ = current_time;

  auto output = ::src::control::loops::flight_loop_queue.output.MakeMessage();

  output->gimbal_angle = 20;

  if (!::src::control::loops::flight_loop_queue.goal.get()) {
    ::std::cerr << "NO GOAL!\n";

    const int iterations = phased_loop_.SleepUntilNext();
    if (iterations < 0) {
      std::cout << "SKIPPED ITERATIONS\n";
    }
    return;
  } else {
    output->trigger_takeoff =
        ::src::control::loops::flight_loop_queue.goal->trigger_takeoff;
    output->trigger_hold =
        ::src::control::loops::flight_loop_queue.goal->trigger_hold;
    output->trigger_offboard =
        ::src::control::loops::flight_loop_queue.goal->trigger_offboard;
    output->trigger_rtl =
        ::src::control::loops::flight_loop_queue.goal->trigger_rtl;
    output->trigger_land =
        ::src::control::loops::flight_loop_queue.goal->trigger_land;
    output->trigger_arm =
        ::src::control::loops::flight_loop_queue.goal->trigger_arm;
    output->trigger_disarm =
        ::src::control::loops::flight_loop_queue.goal->trigger_disarm;
  }

  if (::src::control::loops::flight_loop_queue.goal->trigger_failsafe) {
    EndFlightTimer();
    next_state = FAILSAFE;
  }

  if (::src::control::loops::flight_loop_queue.goal->trigger_throttle_cut) {
    EndFlightTimer();
    next_state = FLIGHT_TERMINATION;
  }

  if (::src::control::loops::flight_loop_queue.goal->trigger_alarm + 0.05 >
      current_time) {
    if (!did_alarm_) {
      did_alarm_ = true;
      alarm_.AddAlert({0.30, 0.30});
      LOG_LINE("Alarm was manually triggered");
    }
  } else {
    did_alarm_ = false;
  }

  // Check if the Pixhawk just got armed, and send out a chirp if it did.
  if(::src::control::loops::flight_loop_queue.sensors->armed && !did_arm_) {
    did_arm_ = true;

    alarm_.AddAlert({0.06, 0.15});
    alarm_.AddAlert({0.06, 0.15});
  }

  if(!::src::control::loops::flight_loop_queue.sensors->armed) {
    did_arm_ = false;
  }

  // Set defaults for all outputs.
  output->velocity_x = 0;
  output->velocity_y = 0;
  output->velocity_z = 0;

  output->alarm = false;

  bool run_mission = ::src::control::loops::flight_loop_queue.goal->run_mission;

  switch (state_) {
    case STANDBY:
      if (run_mission) {
        next_state = ARMING;
      }
      break;

    case ARMING:
      // Check if we have GPS.
      if (::src::control::loops::flight_loop_queue.sensors->last_gps <
          current_time - 0.5) {
        LOG_LINE("can't arm; no GPS (last gps: "
                 << ::src::control::loops::flight_loop_queue.sensors->last_gps
                 << " current time: " << current_time);

        next_state = STANDBY;
        break;
      }

      if (!run_mission) {
        next_state = LANDING;
        break;
      }

      if (::src::control::loops::flight_loop_queue.sensors->armed) {
        next_state = ARMED;
        break;
      }
      break;

    case ARMED:
      if (!run_mission) {
        next_state = LANDING;
        break;
      }

      if (!::src::control::loops::flight_loop_queue.sensors->armed) {
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

      if (!::src::control::loops::flight_loop_queue.sensors->armed) {
        takeoff_ticker_ = 0;
        next_state = ARMING;
        break;
      }

      if (::src::control::loops::flight_loop_queue.sensors->relative_altitude <
          0.3) {
        takeoff_ticker_++;
      }

      if (::src::control::loops::flight_loop_queue.sensors->relative_altitude >
          2.2) {
        takeoff_ticker_ = 0;
        next_state = IN_AIR;
      }
      break;

    case IN_AIR: {
      //    if (!run_mission) {
      //      next_state = LANDING;
      //      break;
      //    }

      //    // Check if altitude is below a safe threshold, which may indicate
      //    that
      //    // the autopilot was reset.
      //    if
      //    (::src::control::loops::flight_loop_queue.sensors->relative_altitude
      //    >
      //            2.2 &&
      //        ::src::control::loops::flight_loop_queue.sensors->relative_altitude
      //        <
      //            2.5) {
      //      next_state = TAKING_OFF;
      //    } else if (::src::control::loops::flight_loop_queue.sensors
      //                   ->relative_altitude < 2.2) {
      //      next_state = LANDING;
      //    }

      Position3D position = {
          ::src::control::loops::flight_loop_queue.sensors->latitude,
          ::src::control::loops::flight_loop_queue.sensors->longitude,
          ::src::control::loops::flight_loop_queue.sensors->relative_altitude};

      pilot::PilotOutput flight_direction = pilot_.Calculate(position);

      output->velocity_x = flight_direction.flight_velocities.x;
      output->velocity_y = flight_direction.flight_velocities.y;
      output->velocity_z = flight_direction.flight_velocities.z;
      break;
    }

    case LANDING:
      if (!::src::control::loops::flight_loop_queue.sensors->armed) {
        EndFlightTimer();
        next_state = STANDBY;
        break;
      }

      if (::src::control::loops::flight_loop_queue.goal->run_mission &&
          ::src::control::loops::flight_loop_queue.sensors->relative_altitude >
              5.0) {
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
  if (next_state == IN_AIR &&
      ::src::control::loops::flight_loop_queue.sensors->last_gps <
          current_time - 0.5) {
    LOG_LINE("no GPS; landing (last gps: "
             << ::src::control::loops::flight_loop_queue.sensors->last_gps
             << " current time: " << current_time);

    next_state = LANDING;
  }

  if (next_state != state_) {
    // Handle state transitions.
    LOG_LINE("Switching states: " << state_ << " -> " << next_state);
  }

  state_ = next_state;

  output->alarm = alarm_.ShouldAlarm();
  LOG_LINE("Flight loop iteration OUTPUT..."
           << " VelocityX: " << output->velocity_x << " VelocityY: "
           << output->velocity_y << " VelocityZ: " << output->velocity_z);

  output.Send();

  auto status = ::src::control::loops::flight_loop_queue.status.MakeMessage();
  status->state = next_state;
  status->current_command_index = 0;

  if (current_flight_start_time_ == 0) {
    status->flight_time = previous_flights_time_;
  } else {
    status->flight_time =
        previous_flights_time_ +
        (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
             .count() -
         current_flight_start_time_);
  }

  LOG_LINE("Flight loop iteration STATUS... "
           << " State: " << status->state
           << " FlightTime: " << status->flight_time
           << " CurrentCommandIndex: " << status->current_command_index);

  status.Send();

  const int iterations = phased_loop_.SleepUntilNext();
  if (iterations < 0) {
    std::cout << "SKIPPED ITERATIONS\n";
  }
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

}  // namespace loops
}  // namespace control
}  // namespace src
