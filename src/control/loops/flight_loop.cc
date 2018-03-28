#include "flight_loop.h"

#include "src/control/loops/flight_loop.q.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "zmq.hpp"

namespace src {
namespace control {
namespace loops {

FlightLoop::FlightLoop()
    : state_(STANDBY),
      running_(false),
      phased_loop_(std::chrono::milliseconds(10), std::chrono::milliseconds(0)),
      start_(std::chrono::system_clock::now()),
      takeoff_ticker_(0),
      verbose_(false),
      count_(0),
      previous_flights_time_(0),
      current_flight_start_time_(0) {
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
  std::chrono::time_point<std::chrono::system_clock> end =
      std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start_;

  ::std::cout
      << "ITERATE in state " << state_ << ::std::endl
      << elapsed.count() << " " << std::setprecision(12)
      << ::src::control::loops::flight_loop_queue.sensors->latitude << ", "
      << ::src::control::loops::flight_loop_queue.sensors->longitude
      << " @ alt "
      << ::src::control::loops::flight_loop_queue.sensors->altitude
      << " @ rel_alt "
      << ::src::control::loops::flight_loop_queue.sensors->relative_altitude
      << std::endl;
  ::std::cout
      << ::src::control::loops::flight_loop_queue.sensors->accelerometer_x
      << ", "
      << ::src::control::loops::flight_loop_queue.sensors->accelerometer_y
      << ", "
      << ::src::control::loops::flight_loop_queue.sensors->accelerometer_z
      << ::std::endl;

  ::std::cout << ::src::control::loops::flight_loop_queue.sensors->gyro_x
              << ", "
              << ::src::control::loops::flight_loop_queue.sensors->gyro_y
              << ", "
              << ::src::control::loops::flight_loop_queue.sensors->gyro_z
              << ::std::endl;

  ::std::cout
      << "abs "
      << ::src::control::loops::flight_loop_queue.sensors->absolute_pressure
      << " relative_pressure "
      << ::src::control::loops::flight_loop_queue.sensors->relative_pressure
      << " pressure_altitude "
      << ::src::control::loops::flight_loop_queue.sensors->pressure_altitude
      << " temperature "
      << ::src::control::loops::flight_loop_queue.sensors->temperature
      << ::std::endl;

  ::std::cout
      << ::src::control::loops::flight_loop_queue.sensors->battery_voltage
      << " volts | current "
      << ::src::control::loops::flight_loop_queue.sensors->battery_current
      << ::std::endl;

  ::std::cout << "armed "
              << ::src::control::loops::flight_loop_queue.sensors->armed
              << ::std::endl;

  if (::src::control::loops::flight_loop_queue.goal.get()) {
    ::std::cout
        << "goal run_mission "
        << ::src::control::loops::flight_loop_queue.goal->run_mission
        << " failsafe "
        << ::src::control::loops::flight_loop_queue.goal->trigger_failsafe
        << " throttle cut "
        << ::src::control::loops::flight_loop_queue.goal
               ->trigger_throttle_cut
        << ::std::endl
        << ::std::endl;
  }
}

void FlightLoop::SetVerbose(bool verbose) { verbose_ = verbose; }

void FlightLoop::RunIteration() {
  // TODO(comran): Are these two queues synced?
  // TODO(comran): Check for stale queue messages.
  ::src::control::loops::flight_loop_queue.sensors.FetchAnother();
  ::src::control::loops::flight_loop_queue.goal.FetchLatest();

  DumpSensorsPeriodic();

  if (!::src::control::loops::flight_loop_queue.goal.get()) {
    ::std::cerr << "NO GOAL!\n";
    return;
  }

  auto output =
      ::src::control::loops::flight_loop_queue.output.MakeMessage();

  if (::src::control::loops::flight_loop_queue.goal->trigger_failsafe) {
    EndFlightTimer();
    state_ = FAILSAFE;
  }

  if (::src::control::loops::flight_loop_queue.goal->trigger_throttle_cut) {
    EndFlightTimer();
    state_ = FLIGHT_TERMINATION;
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

  bool run_mission =
      ::src::control::loops::flight_loop_queue.goal->run_mission;

  switch (state_) {
    case STANDBY:
      if (run_mission) {
        state_ = ARMING;
      }
      break;

    case ARMING:
      if (!run_mission) {
        state_ = LANDING;
      }

      if (::src::control::loops::flight_loop_queue.sensors->armed) {
        state_ = ARMED;
      }

      output->arm = true;
      break;

    case ARMED:
      if (!run_mission) {
        state_ = LANDING;
      }

      if (!::src::control::loops::flight_loop_queue.sensors->armed) {
        state_ = ARMING;
      }

      current_flight_start_time_ =
          std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      state_ = TAKING_OFF;
      break;

    case TAKING_OFF:
      if (!run_mission) {
        takeoff_ticker_ = 0;
        state_ = LANDING;
      }

      if (!::src::control::loops::flight_loop_queue.sensors->armed) {
        takeoff_ticker_ = 0;
        state_ = ARMING;
      }

      if (::src::control::loops::flight_loop_queue.sensors
              ->relative_altitude < 0.3) {
        takeoff_ticker_++;
      }

      if (takeoff_ticker_ < 800) {
        output->arm = true;
        output->takeoff = true;
      } else {
        output->disarm = true;
      }

      if (::src::control::loops::flight_loop_queue.sensors
              ->relative_altitude > 2.2) {
        takeoff_ticker_ = 0;
        state_ = IN_AIR;
      }
      break;

    case IN_AIR: {
      if (!run_mission) {
        state_ = LANDING;
      }

      // Check if altitude is below a safe threshold, which may indicate that
      // the autopilot was reset.
      if (::src::control::loops::flight_loop_queue.sensors
                  ->relative_altitude > 2.2 &&
          ::src::control::loops::flight_loop_queue.sensors
                  ->relative_altitude < 2.5) {
        state_ = TAKING_OFF;
      } else if (::src::control::loops::flight_loop_queue.sensors
                     ->relative_altitude < 2.2) {
        state_ = LANDING;
      }

      Position3D position = {
          ::src::control::loops::flight_loop_queue.sensors->latitude,
          ::src::control::loops::flight_loop_queue.sensors->longitude,
          ::src::control::loops::flight_loop_queue.sensors
              ->relative_altitude};

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
        state_ = STANDBY;
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

  output.Send();

  auto status =
      ::src::control::loops::flight_loop_queue.status.MakeMessage();
  status->state = state_;
  if (current_flight_start_time_ == 0) {
    status->flight_time = previous_flights_time_;
  } else {
    status->flight_time = previous_flights_time_ + (
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() -
        current_flight_start_time_);
  }
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
        std::chrono::system_clock::now().time_since_epoch()).count() -
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
