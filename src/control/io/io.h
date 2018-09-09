#pragma once

#include "src/control/io/autopilot_interface/autopilot_interface.h"

#include "src/control/io/loop_input_handler.h"
#include "src/control/io/loop_output_handler.h"

#include <atomic>
#include <iomanip>
#include <iostream>
#include <limits>
#include <unistd.h>

#ifdef UAS_AT_UCLA_DEPLOYMENT
#include <pigpiod_if2.h>
#include <wiringPi.h>
#endif

#include "lib/dslr_interface/dslr_interface.h"
#include "lib/logger/log_sender.h"
#include "lib/proto_comms/proto_comms.h"
#include "src/control/messages.pb.h"

namespace src {
namespace control {
namespace io {
namespace {
const int kAlarmGPIOPin = 2;
const int kGimbalGPIOPin = 18;
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

class AutopilotSensorReader : public LoopInputHandler {
 public:
  AutopilotSensorReader(autopilot_interface::AutopilotInterface *copter_io);

 private:
  void RunIteration();
  autopilot_interface::AutopilotInterface *copter_io_;
  autopilot_interface::TimeStamps last_timestamps_;

  double last_gps_;

  ::lib::proto_comms::ProtoSender<::src::control::Sensors> sensors_sender_;
};

class AutopilotOutputWriter : public LoopOutputHandler {
 public:
  AutopilotOutputWriter(autopilot_interface::AutopilotInterface *copter_io);

 private:
  virtual void Read() override;
  virtual void Write() override;
  virtual void Stop() override;

#ifdef UAS_AT_UCLA_DEPLOYMENT
  int pigpio_;

  int camera_script_pid_;
  int camera_script_run_;
#endif

  ::lib::DSLRInterface dslr_interface_;

  autopilot_interface::AutopilotInterface *copter_io_;

  bool did_takeoff_;
  bool did_hold_;
  bool did_offboard_;
  bool did_rtl_;
  bool did_land_;
  bool did_arm_;
  bool did_disarm_;

  ::lib::proto_comms::ProtoReceiver<::src::control::Output> output_receiver_;
};

class IO {
 public:
  IO();
  void Run();
  void Quit();

 private:
  autopilot_interface::AutopilotInterface copter_io_;
  ::std::atomic<bool> run_{true};

  AutopilotSensorReader autopilot_sensor_reader_;
  AutopilotOutputWriter autopilot_output_writer_;
};

} // namespace io
} // namespace control
} // namespace src
