#ifndef SPINNY_CONTROL_IO_IO_H_
#define SPINNY_CONTROL_IO_IO_H_

#include "src/control/io/autopilot_interface/autopilot_interface.h"

#include "src/control/io/loop_output_handler.h"
#include "src/control/io/loop_input_handler.h"

#include <atomic>

#ifdef UAS_AT_UCLA_DEPLOYMENT
#include <wiringPi.h>
#endif

#include "src/control/loops/flight_loop.q.h"
#include "aos/common/util/phased_loop.h"

namespace src {
namespace control {
namespace io {
namespace {
const int kAlarmGPIOPin = 2;
}  // namespace

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
};

class AutopilotOutputWriter : public LoopOutputHandler {
 public:
  AutopilotOutputWriter(autopilot_interface::AutopilotInterface *copter_io);

 private:
  virtual void Read() override;
  virtual void Write() override;
  virtual void Stop() override;

  autopilot_interface::AutopilotInterface *copter_io_;

  bool did_takeoff_;
  bool did_hold_;
  bool did_offboard_;
  bool did_rtl_;
  bool did_land_;
  bool did_arm_;
  bool did_disarm_;
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

}  // namespace io
}  // namespace control
}  // namespace src

#endif  // SPINNY_CONTROL_IO_IO_H_
