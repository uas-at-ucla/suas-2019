#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "zmq.hpp"
#include "ros/ros.h"
#include <boost/algorithm/string.hpp>
#include <google/protobuf/text_format.h>

#include "lib/alarm/alarm.h"
#include "lib/logger/log_sender.h"
#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/physics_structs/physics_structs.h"
#include "lib/proto_comms/proto_comms.h"
#include "src/controls/ground_server/timeline/executor/executor.h"
#include "src/controls/loops/state_machine/state_machine.h"
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
namespace loops {

namespace {
static const int kFlightLoopFrequency = 1e2;
static const int kMaxMessageInQueues = 5;
static constexpr double kDefaultGimbalAngle = 0.15;
} // namespace

class FlightLoop {
 public:
  FlightLoop();

  void Run();
  ::src::controls::Output RunIteration(::src::controls::Sensors sensors,
                                       ::src::controls::Goal goal);

 private:
  void DumpProtobufMessages(::src::controls::Sensors &sensors,
                            ::src::controls::Goal &goal,
                            ::src::controls::Output &output);
  void LogProtobufMessage(::std::string name,
                          ::google::protobuf::Message *message);
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

  // Fields ////////////////////////////////////////////////////////////////////
  state_machine::StateMachine state_machine_;

  ::std::atomic<bool> running_;
  ::ros::Rate phased_loop_;

  ::std::chrono::time_point<std::chrono::system_clock> start_;

  ::lib::alarm::Alarm alarm_;

  double last_loop_;
  bool did_alarm_;
  bool did_arm_;

  double last_bomb_drop_;
  double last_dslr_;

  ::lib::proto_comms::ProtoReceiver<::src::controls::UasMessage>
      sensors_receiver_;
  ::lib::proto_comms::ProtoReceiver<::src::controls::UasMessage> goal_receiver_;
  ::lib::proto_comms::ProtoSender<::src::controls::UasMessage> output_sender_;
};

} // namespace loops
} // namespace controls
} // namespace src
