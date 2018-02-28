#include "flight_loop.h"

#include "src/control/loops/flight_loop.q.h"

#include <chrono>
#include <iomanip>
#include <iostream>

namespace spinny {
namespace control {
namespace loops {

FlightLoop::FlightLoop()
    : state_(STANDBY),
      running_(false),
      phased_loop_(std::chrono::milliseconds(10), std::chrono::milliseconds(0)),
      start_(std::chrono::system_clock::now()),
      takeoff_ticker_(0),
      verbose_(false),
      count_(0) {
  ::spinny::control::loops::flight_loop_queue.sensors.FetchLatest();
  ::spinny::control::loops::flight_loop_queue.goal.FetchLatest();
  ::spinny::control::loops::flight_loop_queue.output.FetchLatest();
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
      << ::spinny::control::loops::flight_loop_queue.sensors->latitude << ", "
      << ::spinny::control::loops::flight_loop_queue.sensors->longitude
      << " @ alt "
      << ::spinny::control::loops::flight_loop_queue.sensors->altitude
      << " @ rel_alt "
      << ::spinny::control::loops::flight_loop_queue.sensors->relative_altitude
      << std::endl;
  ::std::cout
      << ::spinny::control::loops::flight_loop_queue.sensors->accelerometer_x
      << ", "
      << ::spinny::control::loops::flight_loop_queue.sensors->accelerometer_y
      << ", "
      << ::spinny::control::loops::flight_loop_queue.sensors->accelerometer_z
      << ::std::endl;

  ::std::cout << ::spinny::control::loops::flight_loop_queue.sensors->gyro_x
              << ", "
              << ::spinny::control::loops::flight_loop_queue.sensors->gyro_y
              << ", "
              << ::spinny::control::loops::flight_loop_queue.sensors->gyro_z
              << ::std::endl;

  ::std::cout
      << "abs "
      << ::spinny::control::loops::flight_loop_queue.sensors->absolute_pressure
      << " relative_pressure "
      << ::spinny::control::loops::flight_loop_queue.sensors->relative_pressure
      << " pressure_altitude "
      << ::spinny::control::loops::flight_loop_queue.sensors->pressure_altitude
      << " temperature "
      << ::spinny::control::loops::flight_loop_queue.sensors->temperature
      << ::std::endl;

  ::std::cout
      << ::spinny::control::loops::flight_loop_queue.sensors->battery_voltage
      << " volts | current "
      << ::spinny::control::loops::flight_loop_queue.sensors->battery_current
      << ::std::endl;

  ::std::cout << "armed "
              << ::spinny::control::loops::flight_loop_queue.sensors->armed
              << ::std::endl;

  if (::spinny::control::loops::flight_loop_queue.goal.get()) {
    ::std::cout
        << "goal run_mission "
        << ::spinny::control::loops::flight_loop_queue.goal->run_mission
        << " failsafe "
        << ::spinny::control::loops::flight_loop_queue.goal->trigger_failsafe
        << " throttle cut "
        << ::spinny::control::loops::flight_loop_queue.goal
               ->trigger_throttle_cut
        << ::std::endl
        << ::std::endl;
  }
}

void FlightLoop::SetVerbose(bool verbose) {
  ::std::cout << "SETTING VERBOSE: " << (verbose ? "true" : "false")
              << ::std::endl;

  verbose_ = verbose;
}

void FlightLoop::RunIteration() {
  // TODO(comran): Are these two queues synced?
  // TODO(comran): Check for stale queue messages.
  ::spinny::control::loops::flight_loop_queue.sensors.FetchAnother();
  ::spinny::control::loops::flight_loop_queue.goal.FetchLatest();

  DumpSensorsPeriodic();

  if (!::spinny::control::loops::flight_loop_queue.goal.get()) {
    ::std::cerr << "NO GOAL!\n";
    return;
  }

  auto output =
      ::spinny::control::loops::flight_loop_queue.output.MakeMessage();

  if (::spinny::control::loops::flight_loop_queue.goal->trigger_failsafe) {
    state_ = FAILSAFE;
  }

  if (::spinny::control::loops::flight_loop_queue.goal->trigger_throttle_cut) {
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
      ::spinny::control::loops::flight_loop_queue.goal->run_mission;

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

      if (::spinny::control::loops::flight_loop_queue.sensors->armed) {
        state_ = ARMED;
      }

      output->arm = true;
      break;

    case ARMED:
      if (!run_mission) {
        state_ = LANDING;
      }

      if (!::spinny::control::loops::flight_loop_queue.sensors->armed) {
        state_ = ARMING;
      }

      state_ = TAKING_OFF;
      break;

    case TAKING_OFF:
      if (!run_mission) {
        takeoff_ticker_ = 0;
        state_ = LANDING;
      }

      if (!::spinny::control::loops::flight_loop_queue.sensors->armed) {
        takeoff_ticker_ = 0;
        state_ = ARMING;
      }

      if (takeoff_ticker_++ < 500) {
        output->arm = true;
        output->takeoff = true;
      } else {
        output->disarm = true;
      }

      if (::spinny::control::loops::flight_loop_queue.sensors
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
      if (::spinny::control::loops::flight_loop_queue.sensors
                  ->relative_altitude > 2.2 &&
          ::spinny::control::loops::flight_loop_queue.sensors
                  ->relative_altitude < 2.5) {
        state_ = TAKING_OFF;
      } else if (::spinny::control::loops::flight_loop_queue.sensors
                     ->relative_altitude < 2.2) {
        state_ = LANDING;
      }

      Position3D position = {
          ::spinny::control::loops::flight_loop_queue.sensors->latitude,
          ::spinny::control::loops::flight_loop_queue.sensors->longitude,
          ::spinny::control::loops::flight_loop_queue.sensors
              ->relative_altitude};

      Vector3D flight_direction = pilot_.Calculate(position);

      output->velocity_x = flight_direction.x;
      output->velocity_y = flight_direction.y;
      output->velocity_z = flight_direction.z;

      output->velocity_control = true;
      break;
    }

    case LANDING:
      if (!::spinny::control::loops::flight_loop_queue.sensors->armed) {
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

  const int iterations = phased_loop_.SleepUntilNext();
  if (iterations < 0) {
    std::cout << "SKIPPED ITERATIONS\n";
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
}  // namespace spinny
