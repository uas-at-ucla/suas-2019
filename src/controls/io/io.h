#pragma once

#include <atomic>
#include <iomanip>
#include <iostream>
#include <limits>
#include <unistd.h>

#ifdef UAS_AT_UCLA_DEPLOYMENT
#include <pigpiod_if2.h>
#include <wiringPi.h>
#endif

#include <ros/ros.h>

#include "lib/dslr_interface/dslr_interface.h"
#include "lib/logger/log_sender.h"
#include "lib/proto_comms/proto_comms.h"
#include "lib/trigger/trigger.h"

#include "src/controls/io/autopilot_interface/autopilot_interface.h"
#include "src/controls/io/loop_handler.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace io {
namespace {
// Output GPIO pins for sending out servo PWM signals to control actuators.
const int kAlarmGPIOPin = 2;
const int kGimbalGPIOPin = 18;

// Tolerance for the time period to accept a trigger signal edge, in seconds.
const double kTriggerSignalTolerance = 0.1;

// Bit patterns for the custom modes used by the PX4 flight controller.
const int kTakeoffCommandMode = 0b00000010000001000000000000000000;
const int kHoldCommandMode = 0b00000100000001000000000000000000;
const int kHoldAlternateCommandMode = 0b00000011000001000000000000000000;
const int kOffboardCommandMode = 0b00000000000001100000000000000000;
const int kRtlCommandMode = 0b00000101000001000000000000000000;
const int kLandCommandMode = 0b00000110000001000000000000000000;

} // namespace

enum AutopilotState {
  UNKNOWN = 0,
  TAKEOFF = 1,
  HOLD = 2,
  OFFBOARD = 3,
  RTL = 4,
  LAND = 5,
};

void quit_handler(int sig);

class AutopilotOutputWriter : public LoopHandler {
 public:
  AutopilotOutputWriter(autopilot_interface::AutopilotInterface *copter_io);

 private:
  virtual void RunIteration() override;
  virtual void Stop() override;

#ifdef UAS_AT_UCLA_DEPLOYMENT
  int pigpio_;

  int camera_script_pid_;
  int camera_script_run_;
#endif

  ::lib::DSLRInterface dslr_interface_;

  autopilot_interface::AutopilotInterface *autopilot_interface_;

  ::lib::proto_comms::ProtoReceiver<::src::controls::Output> output_receiver_;

  ::lib::trigger::Trigger takeoff_trigger_;
  ::lib::trigger::Trigger hold_trigger_;
  ::lib::trigger::Trigger offboard_trigger_;
  ::lib::trigger::Trigger rtl_trigger_;
  ::lib::trigger::Trigger land_trigger_;
  ::lib::trigger::Trigger arm_trigger_;
  ::lib::trigger::Trigger disarm_trigger_;
};

class IO {
 public:
  IO(const char *drone_address);
  void Run();
  void Quit();

 private:
  autopilot_interface::AutopilotInterface autopilot_interface_;
  ::std::atomic<bool> run_{true};

  // AutopilotSensorReader autopilot_sensor_reader_;
  AutopilotOutputWriter autopilot_output_writer_;
};

} // namespace io
} // namespace controls
} // namespace src
