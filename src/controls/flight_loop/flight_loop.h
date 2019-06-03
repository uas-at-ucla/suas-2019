#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

// #include "zmq.hpp"
#include <boost/algorithm/string.hpp>
#include <google/protobuf/text_format.h>
#include <ros/ros.h>

#include "flight_state_machine.h"
#include "lib/alarm/alarm.h"
// #include "lib/mission_manager/mission_commands.pb.h"
// #include "lib/physics_structs/physics_structs.h"
#include "src/controls/ground_controls/timeline/executor/executor.h"
#include "src/controls/io/io.h"
#include "src/controls/messages.pb.h"

/*      ________  ________  ___  ________   ________       ___    ___
       |\   ____\|\   __  \|\  \|\   ___  \|\   ___  \    |\  \  /  /|
       \ \  \___|\ \  \|\  \ \  \ \  \\ \  \ \  \\ \  \   \ \  \/  / /
        \ \_____  \ \   ____\ \  \ \  \\ \  \ \  \\ \  \   \ \    / /
         \|____|\  \ \  \___|\ \  \ \  \\ \  \ \  \\ \  \   \/  /  /
           ____\_\  \ \__\    \ \__\ \__\\ \__\ \__\\ \__\__/  / /
          |\_________\|__|     \|__|\|__| \|__|\|__| \|__|\___/ /
          \|_________|       UAS @ UCLA 2017 - 2019      \|___|/              */

namespace src {
namespace controls {
namespace flight_loop {

namespace {
static constexpr double kDefaultGimbalAngle = 0.15;
static constexpr double kExpectedFlightLoopHz = 50.0;
static constexpr double kFlightLoopTolerancePeriod = 0.001;
static constexpr double kProtobufLogHz = 4;
} // namespace

class FlightLoop {
 public:
  FlightLoop();

  void Run();
  void RunIteration(::src::controls::Sensors sensors);

 private:
  void LogProtobufMessage(::std::string name,
                          ::google::protobuf::Message &message);
  void MonitorLoopFrequency(::src::controls::Sensors);
  void EndFlightTimer();
  ::src::controls::Output GenerateDefaultOutput();

  // Pass through any triggers for actuators on the drone, such as the alarm,
  // gimbal, and drop controls.
  void WriteActuators(::src::controls::Sensors &sensors,
                      ::src::controls::Goal &goal,
                      ::src::controls::Output &output);

  // Calls the appropriate handler based on the current state of the loop.
  void RouteToCurrentState(::src::controls::Sensors &sensors,
                           ::src::controls::Goal &goal,
                           ::src::controls::Output &output);

  // Receive drone program
  void DroneProgramReceived(
      ::src::controls::ground_controls::timeline::DroneProgram drone_program);

  ::src::controls::Goal GetGoal();

  // Fields ////////////////////////////////////////////////////////////////////
  flight_state_machine::FlightStateMachine state_machine_;

  ::std::atomic<bool> running_;

  ::std::chrono::time_point<std::chrono::system_clock> start_;

  double last_loop_;
  double last_proto_log_;
  bool did_alarm_;
  bool did_arm_;

  double last_bomb_drop_;
  double last_dslr_;

  ::src::controls::Goal goal_;
  ::std::mutex goal_mutex_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber sensors_subscriber_;
  ::ros::Subscriber drone_program_subscriber_;
  ::ros::Publisher output_publisher_;
};

} // namespace flight_loop
} // namespace controls
} // namespace src
