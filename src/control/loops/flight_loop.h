#pragma once

#include <atomic>

#include "lib/alarm/alarm.h"
#include "lib/logger/log_sender.h"
#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/physics_structs/physics_structs.h"
#include "lib/proto_comms/proto_comms.h"
#include "src/control/loops/pilot/pilot.h"
#include "src/control/messages.pb.h"

/*      ________  ________  ___  ________   ________       ___    ___
       |\   ____\|\   __  \|\  \|\   ___  \|\   ___  \    |\  \  /  /|
       \ \  \___|\ \  \|\  \ \  \ \  \\ \  \ \  \\ \  \   \ \  \/  / /
        \ \_____  \ \   ____\ \  \ \  \\ \  \ \  \\ \  \   \ \    / /
         \|____|\  \ \  \___|\ \  \ \  \\ \  \ \  \\ \  \   \/  /  /
           ____\_\  \ \__\    \ \__\ \__\\ \__\ \__\\ \__\__/  / /
          |\_________\|__|     \|__|\|__| \|__|\|__| \|__|\___/ /
          \|_________|           UAS @ UCLA 2018         \|___|/              */

namespace src {
namespace control {
namespace loops {

class FlightLoop {
 public:
  FlightLoop();

  void Run();
  void Iterate();

  // Flight loop state machine states.
  enum State {
    STANDBY = 0,
    ARMING = 1,
    ARMED = 2,
    TAKING_OFF = 3,
    IN_AIR = 4,
    LANDING = 5,
    FAILSAFE = 6,
    FLIGHT_TERMINATION = 7
  };

  State state() const { return state_; }

  void SetVerbose(bool verbose);

 private:
  void RunIteration();

  State state_;

  pilot::Pilot pilot_;

  ::std::atomic<bool> running_;
  ::lib::phased_loop::PhasedLoop phased_loop_;

  ::std::chrono::time_point<std::chrono::system_clock> start_;

  int takeoff_ticker_;
  bool verbose_;

  void EndFlightTimer();
  int previous_flights_time_;
  unsigned long current_flight_start_time_;

  ::lib::alarm::Alarm alarm_;

  bool got_sensors_;
  double last_loop_;
  bool did_alarm_;
  bool did_arm_;

  double last_bomb_drop_;
  double last_dslr_;

  ::lib::proto_comms::ProtoReceiver<::src::control::Sensors> sensors_receiver_;
  ::lib::proto_comms::ProtoReceiver<::src::control::Goal> goal_receiver_;
  ::lib::proto_comms::ProtoSender<::src::control::Status> status_sender_;
  ::lib::proto_comms::ProtoSender<::src::control::Output> output_sender_;
};

const ::std::map<FlightLoop::State, ::std::string> state_string = {
    {FlightLoop::STANDBY, "STANDBY"},
    {FlightLoop::ARMING, "ARMING"},
    {FlightLoop::ARMED, "ARMED"},
    {FlightLoop::TAKING_OFF, "TAKING_OFF"},
    {FlightLoop::IN_AIR, "IN_AIR"},
    {FlightLoop::LANDING, "LANDING"},
    {FlightLoop::FAILSAFE, "FAILSAFE"},
    {FlightLoop::FLIGHT_TERMINATION, "FLIGHT_TERMINATION"}};

} // namespace loops
} // namespace control
} // namespace src
