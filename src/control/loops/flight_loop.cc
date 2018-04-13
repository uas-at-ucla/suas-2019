#include "flight_loop.h"

#include <chrono>
#include <iomanip>
#include <iostream>
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
      alarm_(kFlightLoopFrequency) {
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

  DumpSensors();

  State next_state = state_;

  if (!::src::control::loops::flight_loop_queue.goal.get()) {
    ::std::cerr << "NO GOAL!\n";
    return;
  }

  auto output = ::src::control::loops::flight_loop_queue.output.MakeMessage();

  if (::src::control::loops::flight_loop_queue.goal->trigger_failsafe) {
    EndFlightTimer();
    next_state = FAILSAFE;
  }

  if (::src::control::loops::flight_loop_queue.goal->trigger_throttle_cut) {
    EndFlightTimer();
    next_state = FLIGHT_TERMINATION;
  }

  // Set defaults for all outputs.
  output->velocity_x = 0;
  output->velocity_y = 0;
  output->velocity_z = 0;

  output->velocity_control = false;
  output->arm = false;
  output->takeoff = false;
  output->land = false;
  output->throttle_cut = false;

  output->alarm = false;

  bool run_mission = ::src::control::loops::flight_loop_queue.goal->run_mission;

  switch (state_) {
    case STANDBY:
      if (run_mission) {
        next_state = ARMING;
      }
      break;

    case ARMING:
      if (!run_mission) {
        next_state = LANDING;
      }

      if (::src::control::loops::flight_loop_queue.sensors->armed) {
        next_state = ARMED;
      }

      output->arm = true;
      break;

    case ARMED:
      if (!run_mission) {
        next_state = LANDING;
      }

      if (!::src::control::loops::flight_loop_queue.sensors->armed) {
        next_state = ARMING;
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
      }

      if (!::src::control::loops::flight_loop_queue.sensors->armed) {
        takeoff_ticker_ = 0;
        next_state = ARMING;
      }

      if (::src::control::loops::flight_loop_queue.sensors->relative_altitude <
          0.3) {
        takeoff_ticker_++;
      }

      if (takeoff_ticker_ < 800) {
        output->arm = true;
        output->takeoff = true;
      } else {
        output->disarm = true;
      }

      if (::src::control::loops::flight_loop_queue.sensors->relative_altitude >
          2.2) {
        takeoff_ticker_ = 0;
        next_state = IN_AIR;
      }
      break;

    case IN_AIR: {
      if (!run_mission) {
        next_state = LANDING;
      }

      // Check if altitude is below a safe threshold, which may indicate that
      // the autopilot was reset.
      if (::src::control::loops::flight_loop_queue.sensors->relative_altitude >
              2.2 &&
          ::src::control::loops::flight_loop_queue.sensors->relative_altitude <
              2.5) {
        next_state = TAKING_OFF;
      } else if (::src::control::loops::flight_loop_queue.sensors
                     ->relative_altitude < 2.2) {
        next_state = LANDING;
      }

      Position3D position = {
          ::src::control::loops::flight_loop_queue.sensors->latitude,
          ::src::control::loops::flight_loop_queue.sensors->longitude,
          ::src::control::loops::flight_loop_queue.sensors->relative_altitude};

      pilot::PilotOutput flight_direction = pilot_.Calculate(position);

      output->velocity_x = flight_direction.flight_velocities.x;
      output->velocity_y = flight_direction.flight_velocities.y;
      output->velocity_z = flight_direction.flight_velocities.z;

      output->velocity_control = true;
      break;
    }

    case LANDING:
      if (!::src::control::loops::flight_loop_queue.sensors->armed) {
        EndFlightTimer();
        next_state = STANDBY;
      }

      if (::src::control::loops::flight_loop_queue.goal->run_mission &&
          ::src::control::loops::flight_loop_queue.sensors->relative_altitude >
              3.0) {
        next_state = IN_AIR;
      }

      output->land = true;
      break;

    case FAILSAFE:
      output->land = true;
      break;

    case FLIGHT_TERMINATION:
      output->throttle_cut = true;
      break;
  }

  if (next_state != state_) {
    // Handle state transitions.
    LOG_LINE("Switching states: " << state_ << " -> " << next_state);

    if (next_state == ARMING) {
      alarm_.AddAlert({0.10, 0.50});
      alarm_.AddAlert({0.10, 0.50});
    }
  }

  state_ = next_state;

  output->alarm = alarm_.ShouldAlarm();
  LOG_LINE("Flight loop iteration OUTPUT..."
           << " VelocityX: " << output->velocity_x << " VelocityY: "
           << output->velocity_y << " VelocityZ: " << output->velocity_z
           << " VelocityControl: " << output->velocity_control
           << " Arm: " << output->arm << " Disarm: " << output->disarm
           << " Takeoff: " << output->takeoff << " Land: " << output->land
           << " ThrottleCut: " << output->throttle_cut
           << " Alarm: " << output->alarm);

  output.Send();

  auto status = ::src::control::loops::flight_loop_queue.status.MakeMessage();
  status->state = next_state;
  status->current_command_index = pilot_.GetCurrentCommandIndex();

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

void receive_mission() {
  ::zmq::context_t context(1);
  ::zmq::socket_t ground_communicator_stream(context, ZMQ_REQ);
  ground_communicator_stream.connect("ipc:///tmp/mission_command_stream.ipc");
  ::zmq::message_t request(5);
  memcpy(request.data(), "Hello", 5);
  ground_communicator_stream.send(request);

  ::std::cout << "-----------------Hello-----------------\n";
  ::zmq::message_t reply;
  ground_communicator_stream.recv(&reply);
  for (int i = 0;; i++)
    if (i % 1000000 == 0)
      ::std::cout << "got a reply <><><><><><><><><><><><><><><\n";
}

void FlightLoop::Run() {
  running_ = true;

  ::std::cout << "<<<<<<<<<<<<<<< Creating new ground_communicator_thread\n";

  ::std::thread ground_communicator_thread(receive_mission);
  ground_communicator_thread.detach();

  ::std::cout << "<<<<<<<<<<<<<<< Detatched ground_communicator_thread\n";

  while (running_) {
    RunIteration();
  }
}

}  // namespace loops
}  // namespace control
}  // namespace src
