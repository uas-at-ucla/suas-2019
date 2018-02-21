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
      count_(0) {

  ::spinny::control::loops::flight_loop_queue.sensors.FetchLatest();
  ::spinny::control::loops::flight_loop_queue.goal.FetchLatest();
  ::spinny::control::loops::flight_loop_queue.output.FetchLatest();
}

void FlightLoop::Iterate() { RunIteration(); }

void FlightLoop::DumpSensorsPeriodic() {
  if(count_++ % 10) return;

  DumpSensors();
}

void FlightLoop::DumpSensors() {
  std::chrono::time_point<std::chrono::system_clock> end =
      std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start_;

  ::std::cout
      << "ITERATE " << elapsed.count() << " " << std::setprecision(12)
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
              << ::std::endl
              << ::std::endl;
}

void FlightLoop::RunIteration() {
  // TODO(comran): Are these two queues synced?
  // TODO(comran): Check for stale queue messages.
  ::spinny::control::loops::flight_loop_queue.sensors.FetchAnother();

  DumpSensorsPeriodic();

  ::spinny::control::loops::flight_loop_queue.goal.FetchLatest();
  if(!::spinny::control::loops::flight_loop_queue.goal.get()) {
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
        state_ = LANDING;
      }

      if (!::spinny::control::loops::flight_loop_queue.sensors->armed) {
        state_ = ARMING;
      }

      output->arm = true;
      output->takeoff = true;

      if (::spinny::control::loops::flight_loop_queue.sensors
              ->relative_altitude > 2.2) {
        state_ = IN_AIR;
      }
      break;

    case IN_AIR:
      if (!run_mission) {
        state_ = LANDING;
      }

      if (::spinny::control::loops::flight_loop_queue.sensors
                  ->relative_altitude > 2.2 &&
          ::spinny::control::loops::flight_loop_queue.sensors
                  ->relative_altitude < 2.5) {
        state_ = TAKING_OFF;
      } else if (::spinny::control::loops::flight_loop_queue.sensors
                     ->relative_altitude < 2.2) {
        state_ = LANDING;
      }

      output->velocity_control = true;
      output->velocity_x = 0.5;
      output->velocity_y = 0.5;
      break;

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
