#pragma once

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>

#include "zmq.hpp"

#include "lib/alarm/alarm.h"
#include "lib/logger/log_sender.h"
#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/physics_structs/physics_structs.h"
#include "lib/proto_comms/proto_comms.h"
#include "src/controls/ground_server/timeline/executor/executor.h"
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
enum State {
  STANDBY,
  ARMING,
  ARMED_WAIT_FOR_SPINUP,
  ARMED,
  TAKING_OFF,
  TAKEN_OFF,
  SAFETY_PILOT_CONTROL,
  AUTOPILOT,
  LANDING,
  FAILSAFE,
  FLIGHT_TERMINATION
};

namespace {
static const int kFlightLoopFrequency = 1e2;
static const int kMaxMessageInQueues = 5;
const ::std::map<State, ::std::string> kStateString = {
    {State::STANDBY, "STANDBY"},
    {ARMING, "ARMING"},
    {ARMED, "ARMED"},
    {TAKING_OFF, "TAKING_OFF"},
    {SAFETY_PILOT_CONTROL, "SAFETY_PILOT_CONTROL"},
    {AUTOPILOT, "AUTOPILOT"},
    {LANDING, "LANDING"},
    {FAILSAFE, "FAILSAFE"},
    {FLIGHT_TERMINATION, "FLIGHT_TERMINATION"}};
} // namespace

class FlightLoop {
 public:
  FlightLoop();

  void Run();
  ::src::controls::Output RunIteration(::src::controls::Sensors sensors,
                                       ::src::controls::Goal goal);

  State state() const { return state_; }

  void SetVerbose(bool verbose);

 private:
  void MonitorLoopFrequency(::src::controls::Sensors);
  void EndFlightTimer();
  ::src::controls::Output GenerateDefaultOutput();

  void HandleStandby(::src::controls::Sensors &sensors,
                     ::src::controls::Goal &goal,
                     ::src::controls::Output &output);
  void HandleArming(::src::controls::Sensors &sensors,
                    ::src::controls::Goal &goal,
                    ::src::controls::Output &output);
  void HandleArmedWaitForSpinup(::src::controls::Sensors &sensors,
                                ::src::controls::Goal &goal,
                                ::src::controls::Output &output);
  void HandleArmed(::src::controls::Sensors &sensors,
                   ::src::controls::Goal &goal,
                   ::src::controls::Output &output);
  void HandleTakingOff(::src::controls::Sensors &sensors,
                       ::src::controls::Goal &goal,
                       ::src::controls::Output &output);
  void HandleTakenOff(::src::controls::Sensors &sensors,
                      ::src::controls::Goal &goal,
                      ::src::controls::Output &output);
  void HandleSafetyPilotControl(src::controls::Sensors &sensors,
                                ::src::controls::Goal &goal,
                                ::src::controls::Output &output);
  void HandleAutopilot(::src::controls::Sensors &sensors,
                       ::src::controls::Goal &goal,
                       ::src::controls::Output &output);
  void HandleLanding(::src::controls::Sensors &sensors,
                     ::src::controls::Goal &goal,
                     ::src::controls::Output &output);
  void HandleFailsafe(::src::controls::Sensors &sensors,
                      ::src::controls::Goal &goal,
                      ::src::controls::Output &output);
  void HandleFlightTermination(::src::controls::Sensors &sensors,
                               ::src::controls::Goal &goal,
                               ::src::controls::Output &output);

  State state_;

  executor::Executor executor_;

  ::std::atomic<bool> running_;
  ::lib::phased_loop::PhasedLoop phased_loop_;

  ::std::chrono::time_point<std::chrono::system_clock> start_;

  int takeoff_ticker_;
  bool verbose_;

  int previous_flights_time_;
  unsigned long current_flight_start_time_;

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
