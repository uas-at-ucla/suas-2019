#include "io.h"

namespace src {
namespace control {
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

IO::IO()
#ifdef UAS_AT_UCLA_DEPLOYMENT
    : copter_io_("/dev/ttyS0", 921600),
#else
    : copter_io_("/tmp/virtualcom0", 921600),
#endif
      autopilot_sensor_reader_(&copter_io_),
      autopilot_output_writer_(&copter_io_) {
  copter_io_quit = &copter_io_;
  io_quit = this;
  signal(SIGINT, quit_handler);
}

void IO::Run() {
  copter_io_.start();

  ::std::thread autopilot_sensor_reader_thread(
      ::std::ref(autopilot_sensor_reader_));
  ::std::thread autopilot_output_writer_thread(
      ::std::ref(autopilot_output_writer_));

  autopilot_sensor_reader_thread.join();
  autopilot_output_writer_thread.join();

  copter_io_.stop();
}

void IO::Quit() {
  run_ = false;
  autopilot_sensor_reader_.Quit();
  autopilot_output_writer_.Quit();
}

AutopilotSensorReader::AutopilotSensorReader(
    autopilot_interface::AutopilotInterface *copter_io)
    : copter_io_(copter_io),
      last_gps_(-::std::numeric_limits<double>::infinity()),
      sensors_sender_("ipc:///tmp/uasatucla_sensors->ipc") {
  last_timestamps_.reset_timestamps();
}

void AutopilotSensorReader::RunIteration() {
  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  autopilot_interface::TimeStamps current_timestamps =
      copter_io_->current_messages.time_stamps;

  if (current_timestamps.global_position_int -
      last_timestamps_.global_position_int) {
    last_gps_ = ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
                    ::std::chrono::system_clock::now().time_since_epoch())
                    .count() *
                1e-9;
  }

  last_timestamps_ = current_timestamps;

  ::src::control::Sensors *sensors = new ::src::control::Sensors();

  // GPS data.
  {
    mavlink_global_position_int_t gps =
        copter_io_->current_messages.global_position_int;

    sensors->set_latitude(static_cast<double>(gps.lat) / 1e7);
    sensors->set_longitude(static_cast<double>(gps.lon) / 1e7);
    sensors->set_altitude(static_cast<float>(gps.alt) / 1e3);

    sensors->set_relative_altitude(static_cast<float>(gps.relative_alt) / 1e3);

    sensors->set_velocity_x(static_cast<float>(gps.vx) / 1e2);
    sensors->set_velocity_y(static_cast<float>(gps.vy) / 1e2);
    sensors->set_velocity_z(static_cast<float>(gps.vz) / 1e2);
  }

  sensors->set_heading(
      static_cast<float>(copter_io_->current_messages.vfr_hud.heading));

  // GPS raw data.
  {
    mavlink_gps_raw_int_t gps_raw = copter_io_->current_messages.gps_raw_int;

    sensors->set_gps_satellite_count(gps_raw.satellites_visible);
    sensors->set_gps_eph(gps_raw.eph);
    sensors->set_gps_epv(gps_raw.epv);
    sensors->set_gps_ground_speed(static_cast<float>(gps_raw.vel) / 1e2);
  }

  // IMU data.
  {
    mavlink_highres_imu_t imu = copter_io_->current_messages.highres_imu;

    sensors->set_accelerometer_x(imu.xacc);
    sensors->set_accelerometer_y(imu.yacc);
    sensors->set_accelerometer_z(imu.zacc);

    sensors->set_gyro_x(imu.xgyro);
    sensors->set_gyro_y(imu.ygyro);
    sensors->set_gyro_z(imu.zgyro);

    sensors->set_absolute_pressure(imu.abs_pressure);
    sensors->set_relative_pressure(imu.diff_pressure);
    sensors->set_pressure_altitude(imu.pressure_alt);
    sensors->set_temperature(imu.temperature);
  }

  sensors->set_last_gps(last_gps_);

  // Battery data.
  {
    mavlink_sys_status_t sys_status = copter_io_->current_messages.sys_status;

    sensors->set_battery_voltage(
        static_cast<float>(sys_status.voltage_battery) / 1e3);
    sensors->set_battery_current(
        static_cast<float>(sys_status.current_battery) / 1e3);
  }

  // Armed?
  {
    mavlink_heartbeat_t heartbeat = copter_io_->current_messages.heartbeat;
    sensors->set_armed(!!(heartbeat.base_mode >> 7));

    AutopilotState autopilot_state = UNKNOWN;

    switch (heartbeat.custom_mode) {
      case kTakeoffCommandMode:
        autopilot_state = TAKEOFF;
        break;
      case kHoldCommandMode:
      case kHoldAlternateCommandMode:
        autopilot_state = HOLD;
        break;
      case kOffboardCommandMode:
        autopilot_state = OFFBOARD;
        break;
      case kRtlCommandMode:
        autopilot_state = RTL;
        break;
      case kLandCommandMode:
        autopilot_state = LAND;
        break;
      default:
        autopilot_state = UNKNOWN;
    }

    sensors->set_autopilot_state(autopilot_state);
  }

  // Serialize sensor protobuf and send it over ZMQ.
  UasMessage message = ::src::control::UasMessage();
  message.set_time(current_time);
  message.set_allocated_sensors(sensors);
  sensors_sender_.Send(message);
}

AutopilotOutputWriter::AutopilotOutputWriter(
    autopilot_interface::AutopilotInterface *copter_io)
    : copter_io_(copter_io),
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
  ::src::control::Output output;
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

  copter_io_->update_setpoint(sp);

#ifdef UAS_AT_UCLA_DEPLOYMENT
  digitalWrite(kAlarmGPIOPin, output.alarm() ? HIGH : LOW);

  int bomb_drop_signal = output.bomb_drop() ? 1000 : 1600;

  set_servo_pulsewidth(pigpio_, 23, bomb_drop_signal);

  int gimbal_angle = 1000 + output.gimbal_angle() * 1000;
  set_servo_pulsewidth(pigpio_, 24, gimbal_angle);
#endif

  if (takeoff_trigger_.Process(output.trigger_takeoff())) {
    copter_io_->Takeoff();
  }

  if (hold_trigger_.Process(output.trigger_hold())) {
    copter_io_->Hold();
  }

  if (offboard_trigger_.Process(output.trigger_offboard())) {
    copter_io_->Offboard();
  }

  if (offboard_trigger_.Process(output.trigger_rtl())) {
    copter_io_->ReturnToLaunch();
  }

  if (land_trigger_.Process(output.trigger_land())) {
    copter_io_->Land();
  }

  if (arm_trigger_.Process(output.trigger_arm())) {
    copter_io_->Arm();
  }

  if (disarm_trigger_.Process(output.trigger_disarm())) {
    copter_io_->Disarm();
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
} // namespace control
} // namespace src
