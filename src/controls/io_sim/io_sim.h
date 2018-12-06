#pragma once

#include <atomic>
#include <iomanip>
#include <iostream>
#include <limits>
#include <unistd.h>
#include <mutex>
#include <boost/asio.hpp>

#include "lib/logger/log_sender.h"
#include "lib/proto_comms/proto_comms.h"
#include "lib/trigger/trigger.h"

#include "src/controls/io/loop_handler.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace io {
namespace {
// Tolerance for the time period to accept a trigger signal edge, in seconds.
const double kTriggerSignalTolerance = 0.1;

struct PosInfo {
  double latitude;
  double longitude;
  float altitude;
  float velocity_x;
  float velocity_y;
  float velocity_z;
} pos_info;

std::mutex pos_info_mutex;

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

class AutopilotSensorReader : public LoopHandler {
 public:
  AutopilotSensorReader();

 private:
  virtual void RunIteration() override;
  virtual void Stop() override{};

  double last_gps_;

  ::lib::proto_comms::ProtoSender<::src::controls::UasMessage> sensors_sender_;
};

class AutopilotOutputWriter : public LoopHandler {
 public:
  AutopilotOutputWriter();

 private:
  virtual void RunIteration() override;
  virtual void Stop() override;

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
  IO();
  void Run();
  void Quit();

 private:
  ::std::atomic<bool> run_{true};

  AutopilotSensorReader autopilot_sensor_reader_;
  AutopilotOutputWriter autopilot_output_writer_;
};

} // namespace io
} // namespace controls
} // namespace src
