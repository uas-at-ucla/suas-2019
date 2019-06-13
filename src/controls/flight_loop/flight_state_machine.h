#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <thread>

#include "zmq.hpp"
#include <boost/algorithm/string.hpp>
#include <google/protobuf/text_format.h>
#include <ros/console.h>

#include "lib/alarm/alarm.h"
#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/physics_structs/physics_structs.h"
#include "lib/proto_comms/proto_comms.h"
#include "mission_state_machine.h"
#include "src/controls/constants.h"
#include "src/controls/ground_controls/timeline/timeline_grammar.pb.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace flight_state_machine {
namespace {
// Time between sending arm triggers to the flight controller.
static constexpr double kTriggerPeriod = 5.0;

// Time for propellers to spin up before taking off, in seconds.
static constexpr double kSpinupTime = 5.0;

// Altitude to reach during takeoff before acknowledging the drone has taken
// off.
static constexpr double kMinTakeoffAltitude = 2.0;

static constexpr double kTakenOffWaitTime = 5.0;

} // namespace

enum FlightLoopState {
  STANDBY = 0,
  ARMING = 1,
  ARMED_WAIT_FOR_SPINUP = 2,
  TAKING_OFF = 3,
  TAKEN_OFF = 4,
  MISSION = 5,
  SAFETY_PILOT_CONTROL = 6,
  LANDING = 7,
};

::std::string StateToString(FlightLoopState state);

// States //////////////////////////////////////////////////////////////////////
class State {
 public:
  State() = default;
  virtual ~State() = default;

  virtual void Handle(::src::controls::Sensors &sensors,
                      ::src::controls::Goal &goal,
                      ::src::controls::Output &output) = 0;

  virtual void Reset() = 0;
};

class StandbyState : public State {
 public:
  StandbyState();
  ~StandbyState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

class ArmingState : public State {
 public:
  ArmingState();
  ~ArmingState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;

 private:
  double start_;
  bool was_reset_;
};

class ArmedWaitForSpinupState : public State {
 public:
  ArmedWaitForSpinupState();
  ~ArmedWaitForSpinupState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;

 private:
  double start_;
  bool was_reset_;
};

class TakingOffState : public State {
 public:
  TakingOffState();
  ~TakingOffState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

class TakenOffState : public State {
 public:
  TakenOffState();
  ~TakenOffState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;

 private:
  double start_;
  bool was_reset_;
};

class SafetyPilotControlState : public State {
 public:
  SafetyPilotControlState();
  ~SafetyPilotControlState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

class MissionState : public State {
 public:
  MissionState();
  ~MissionState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
  void LoadMission(
      ::src::controls::ground_controls::timeline::DroneProgram drone_program);

 private:
  mission_state_machine::MissionStateMachine mission_state_machine_;
  double start_;
  bool was_reset_;
};

class LandingState : public State {
 public:
  LandingState();
  ~LandingState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

class UnknownState : public State {
 public:
  UnknownState();
  ~UnknownState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

// State machine router ////////////////////////////////////////////////////////
class FlightStateMachine {
 public:
  FlightStateMachine();
  ~FlightStateMachine();

  void StateTransition(::src::controls::Output &output);

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);

  void LoadMission(
      ::src::controls::ground_controls::timeline::DroneProgram drone_program);

 private:
  State *GetStateHandler(FlightLoopState state);

  FlightLoopState state_;
  ::std::map<FlightLoopState, State *> state_handlers_;
  UnknownState *unknown_state_;
  bool mission_commanded_land_;
};

} // namespace flight_state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src
