#include "flight_loop.h"

#include "src/control/loops/flight_loop.q.h"

#include <chrono>
#include <iomanip>
#include <iostream>

namespace spinny {
namespace control {
namespace loops {

FlightLoop::FlightLoop()
    : state_(UNINITIALIZED),
      running_(false),
      phased_loop_(std::chrono::milliseconds(50),
                   std::chrono::milliseconds(0)) {}

void FlightLoop::RunIteration() {
  if (!::spinny::control::loops::flight_loop_queue.sensors.FetchLatest()) {
    return;
  }

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
      << ::std::endl
      << ::std::endl;

  auto output_message =
      ::spinny::control::loops::flight_loop_queue.output.MakeMessage();

  output_message->velocity_x = 0;
  output_message->velocity_y = 0;
  output_message->velocity_z = 0;

  output_message->velocity_control = false;
  output_message->arm = false;
  output_message->takeoff = false;
  output_message->land = false;
  output_message->throttle_cut = false;

  std::cout << "STATE: " << state_ << std::endl;
  static int i = 0;
  i++;
  switch (state_) {
    case UNINITIALIZED:
      state_ = ARMING;
      break;
    case STANDBY:
      break;
    case ARMING:
      output_message->arm = true;
      if (i > 80) {
        state_ = TAKING_OFF;
      }
      break;
    case ARMED:
      break;
    case TAKING_OFF:
      output_message->arm = true;
      output_message->takeoff = true;
      if (::spinny::control::loops::flight_loop_queue.sensors
              ->relative_altitude > 2.2) {
        state_ = IN_AIR;
      }
      break;
    case IN_AIR:
      output_message->velocity_control = true;
      if(i % 400 < 100) {
        output_message->velocity_y = 10;
      } else if(i % 400 < 200) {
        output_message->velocity_x = 10;
      } else if(i % 400 < 300) {
        output_message->velocity_y = -10;
      } else {
        output_message->velocity_x = -10;
      }
      break;
    case LANDING:
      output_message->land = true;
      break;
    case FAILSAFE:
      output_message->land = true;
      break;
    case FLIGHT_TERMINATION:
      output_message->throttle_cut = true;
      break;
  }

  output_message.Send();
}

void FlightLoop::Run() {
  running_ = true;

  start_ = std::chrono::system_clock::now();

  while (running_) {
    RunIteration();

    const int iterations = phased_loop_.SleepUntilNext();
    if (iterations < 0) {
      std::cout << "SKIPPED ITERATIONS\n";
    }
  }
}

}  // namespace loops
}  // namespace control
}  // namespace spinny
