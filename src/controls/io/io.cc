#include "io.h"

namespace src {
namespace controls {
namespace io {

autopilot_interface::AutopilotInterface *copter_io_quit;
IO *io_quit;
void quit_handler(int sig) {
  io_quit->Quit();

  try {
    copter_io_quit->handle_quit(sig);
  } catch (int error) {
  }
}

IO::IO(const char *drone_address) :
    autopilot_interface_(drone_address),
    autopilot_output_writer_(&autopilot_interface_) {
  copter_io_quit = &autopilot_interface_;
  io_quit = this;
  signal(SIGINT, quit_handler);
}

void IO::Run() {
  autopilot_interface_.start();

  ::std::thread autopilot_output_writer_thread(
      ::std::ref(autopilot_output_writer_));

  autopilot_output_writer_thread.join();
  ::std::cout << "HERE\n";
  autopilot_interface_.stop();
}

void IO::Quit() {
  run_ = false;
  // autopilot_sensor_reader_.Quit();
  autopilot_output_writer_.Quit();
}

AutopilotOutputWriter::AutopilotOutputWriter(
    autopilot_interface::AutopilotInterface *copter_io) :
    autopilot_interface_(copter_io),
    output_receiver_("ipc:///tmp/uasatucla_output.ipc", 5),
    takeoff_trigger_(kTriggerSignalTolerance),
    hold_trigger_(kTriggerSignalTolerance),
    offboard_trigger_(kTriggerSignalTolerance),
    rtl_trigger_(kTriggerSignalTolerance),
    land_trigger_(kTriggerSignalTolerance),
    arm_trigger_(kTriggerSignalTolerance),
    disarm_trigger_(kTriggerSignalTolerance) {

#ifdef UAS_AT_UCLA_DEPLOYMENT
  // Alarm IO setup.
  wiringPiSetup();
  pinMode(kAlarmGPIOPin, OUTPUT);

  // Gimbal IO setup.
  pigpio_ = pigpio_start(0, 0);
  set_mode(pigpio_, 23, PI_OUTPUT);
  set_mode(pigpio_, 24, PI_OUTPUT);
#endif
}

void AutopilotOutputWriter::RunIteration() {
  ::src::controls::Output output;
  if (!output_receiver_.HasMessages()) {
    return;
  }

  output = output_receiver_.GetLatest();

  if (output.dslr()) {
    dslr_interface_.TakePhotos();
  }

  mavlink_set_position_target_local_ned_t sp;

  autopilot_interface::set_velocity(output.velocity_x(), output.velocity_y(),
                                    output.velocity_z(), sp);

  autopilot_interface::set_yaw(output.yaw_setpoint(), sp);

  autopilot_interface_->update_setpoint(sp);

#ifdef UAS_AT_UCLA_DEPLOYMENT
  digitalWrite(kAlarmGPIOPin, output.alarm() ? HIGH : LOW);

  int bomb_drop_signal = output.bomb_drop() ? 1000 : 1600;

  set_servo_pulsewidth(pigpio_, 23, bomb_drop_signal);

  int gimbal_angle = 1000 + output.gimbal_angle() * 1000;
  set_servo_pulsewidth(pigpio_, 24, gimbal_angle);
#endif

  if (takeoff_trigger_.Process(output.trigger_takeoff())) {
    autopilot_interface_->Takeoff();
  }

  if (hold_trigger_.Process(output.trigger_hold())) {
    autopilot_interface_->Hold();
  }

  if (offboard_trigger_.Process(output.trigger_offboard())) {
    autopilot_interface_->Offboard();
  }

  if (offboard_trigger_.Process(output.trigger_rtl())) {
    autopilot_interface_->ReturnToLaunch();
  }

  if (land_trigger_.Process(output.trigger_land())) {
    autopilot_interface_->Land();
  }

  if (arm_trigger_.Process(output.trigger_arm())) {
    autopilot_interface_->Arm();
  }

  if (disarm_trigger_.Process(output.trigger_disarm())) {
    autopilot_interface_->Disarm();
  }
}

void AutopilotOutputWriter::Stop() {
#ifdef UAS_AT_UCLA_DEPLOYMENT
  // Don't leave the alarm on after quitting code.
  digitalWrite(kAlarmGPIOPin, LOW);
#endif

  // No recent output queue messages received, so land drone.
  dslr_interface_.Quit();
}

} // namespace io
} // namespace controls
} // namespace src
