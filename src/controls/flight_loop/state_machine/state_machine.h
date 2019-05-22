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
#include "src/controls/ground_controls/timeline/executor/executor.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace flight_loop {
namespace state_machine {
namespace {
// Time for propellers to spin up before taking off, in seconds.
static constexpr double kSpinupTime = 2.0;

// Altitude to reach during takeoff before acknowledging the drone has taken
// off.
static constexpr double kTakeoffAltitude = 3.0;
} // namespace

enum FlightLoopState {
  STANDBY,
  ARMING,
  ARMED_WAIT_FOR_SPINUP,
  ARMED,
  TAKING_OFF,
  TAKEN_OFF,
  SAFETY_PILOT_CONTROL,
  MISSION,
  LANDING,
  FAILSAFE,
  FLIGHT_TERMINATION,
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
};

class ArmedState : public State {
 public:
  ArmedState();
  ~ArmedState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
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

 private:
  executor::Executor executor_;
};

class LandingState : public State {
 public:
  LandingState();
  ~LandingState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

class FailsafeState : public State {
 public:
  FailsafeState();
  ~FailsafeState() = default;

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output) override;
  void Reset() override;
};

class FlightTerminationState : public State {
 public:
  FlightTerminationState();
  ~FlightTerminationState() = default;

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
class StateMachine {
 public:
  StateMachine();
  ~StateMachine();

  void StateTransition(::src::controls::Output &output);
  bool SafetyStateOverride(::src::controls::Goal &goal,
                           ::src::controls::Output &output);

  void Handle(::src::controls::Sensors &sensors, ::src::controls::Goal &goal,
              ::src::controls::Output &output);

 private:
  State *GetStateHandler(FlightLoopState state);

  FlightLoopState state_;
  ::std::map<FlightLoopState, State *> state_handlers_;
  UnknownState *unknown_state_;
};

} // namespace state_machine
} // namespace flight_loop
} // namespace controls
} // namespace src
