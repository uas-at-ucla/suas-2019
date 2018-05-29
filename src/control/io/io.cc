#include "io.h"

#include <iostream>
#include <limits>

#include "lib/logger/log_sender.h"

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

  exit(0);
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
  signal(SIGTERM, quit_handler);
}

void IO::Run() {
  copter_io_.start();

  ::std::thread autopilot_sensor_reader_thread(
      ::std::ref(autopilot_sensor_reader_));
  ::std::thread autopilot_output_writer_thread(
      ::std::ref(autopilot_output_writer_));

  // Wait forever.
  select(0, nullptr, nullptr, nullptr, nullptr);

  autopilot_sensor_reader_.Quit();
  autopilot_sensor_reader_thread.join();
  autopilot_output_writer_.Quit();
  autopilot_output_writer_thread.join();

  copter_io_.stop();
}

void IO::Quit() {
  run_ = false;
  autopilot_output_writer_.Quit();
}

AutopilotSensorReader::AutopilotSensorReader(
    autopilot_interface::AutopilotInterface *copter_io)
    : copter_io_(copter_io),
      last_gps_(-::std::numeric_limits<double>::infinity()) {
  last_timestamps_.reset_timestamps();
}

void AutopilotSensorReader::RunIteration() {
  // TODO(comran): Fix partial queue writes.
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

  auto flight_loop_sensors_message =
      ::src::control::loops::flight_loop_queue.sensors.MakeMessage();

  // GPS data.
  mavlink_global_position_int_t gps =
      copter_io_->current_messages.global_position_int;

  flight_loop_sensors_message->latitude = static_cast<double>(gps.lat) / 1e7;
  flight_loop_sensors_message->longitude = static_cast<double>(gps.lon) / 1e7;
  flight_loop_sensors_message->altitude = static_cast<float>(gps.alt) / 1e3;

  flight_loop_sensors_message->heading =
      static_cast<float>(copter_io_->current_messages.vfr_hud.heading);

  flight_loop_sensors_message->relative_altitude =
      static_cast<float>(gps.relative_alt) / 1e3;

  flight_loop_sensors_message->velocity_x = static_cast<float>(gps.vx) / 1e2;
  flight_loop_sensors_message->velocity_y = static_cast<float>(gps.vy) / 1e2;
  flight_loop_sensors_message->velocity_z = static_cast<float>(gps.vz) / 1e2;

  // GPS raw data.
  mavlink_gps_raw_int_t gps_raw = copter_io_->current_messages.gps_raw_int;

  flight_loop_sensors_message->gps_satellite_count = gps_raw.satellites_visible;
  flight_loop_sensors_message->gps_eph = gps_raw.eph;
  flight_loop_sensors_message->gps_epv = gps_raw.epv;
  flight_loop_sensors_message->gps_ground_speed =
      static_cast<float>(gps_raw.vel) / 1e2;

  // IMU data.
  mavlink_highres_imu_t imu = copter_io_->current_messages.highres_imu;

  flight_loop_sensors_message->accelerometer_x = imu.xacc;
  flight_loop_sensors_message->accelerometer_y = imu.yacc;
  flight_loop_sensors_message->accelerometer_z = imu.zacc;

  flight_loop_sensors_message->gyro_x = imu.xgyro;
  flight_loop_sensors_message->gyro_y = imu.ygyro;
  flight_loop_sensors_message->gyro_z = imu.zgyro;

  flight_loop_sensors_message->absolute_pressure = imu.abs_pressure;
  flight_loop_sensors_message->relative_pressure = imu.diff_pressure;
  flight_loop_sensors_message->pressure_altitude = imu.pressure_alt;
  flight_loop_sensors_message->temperature = imu.temperature;
  flight_loop_sensors_message->last_gps = last_gps_;

  // Battery data.
  mavlink_sys_status_t sys_status = copter_io_->current_messages.sys_status;
  flight_loop_sensors_message->battery_voltage =
      static_cast<float>(sys_status.voltage_battery) / 1e3;
  flight_loop_sensors_message->battery_current =
      static_cast<float>(sys_status.current_battery) / 1e3;

  // Armed?
  mavlink_heartbeat_t heartbeat = copter_io_->current_messages.heartbeat;
  flight_loop_sensors_message->armed = !!(heartbeat.base_mode >> 7);

  AutopilotState autopilot_state = UNKNOWN;

  switch (heartbeat.custom_mode) {
    case 0b00000010000001000000000000000000:
      autopilot_state = TAKEOFF;
      break;
    case 0b00000100000001000000000000000000:
    case 0b00000011000001000000000000000000:
      autopilot_state = HOLD;
      break;
    case 0b00000000000001100000000000000000:
      autopilot_state = OFFBOARD;
      break;
    case 0b00000101000001000000000000000000:
      autopilot_state = RTL;
      break;
    case 0b00000110000001000000000000000000:
      autopilot_state = LAND;
      break;
    default:
      autopilot_state = UNKNOWN;
  }

  // std::bitset<32> y(heartbeat.custom_mode);
  // std::cout << y << ::std::endl;

  flight_loop_sensors_message->autopilot_state = autopilot_state;

  flight_loop_sensors_message.Send();
}

AutopilotOutputWriter::AutopilotOutputWriter(
    autopilot_interface::AutopilotInterface *copter_io)
    : copter_io_(copter_io) {
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

void AutopilotOutputWriter::Read() {
  ::src::control::loops::flight_loop_queue.output.FetchAnother();
  ::src::control::loops::flight_loop_queue.sensors.FetchAnother();
  ::src::control::loops::flight_loop_queue.goal.FetchLatest();
}

void AutopilotOutputWriter::Write() {
  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  mavlink_set_position_target_local_ned_t sp;

  autopilot_interface::set_velocity(
      ::src::control::loops::flight_loop_queue.output->velocity_x,
      ::src::control::loops::flight_loop_queue.output->velocity_y,
      ::src::control::loops::flight_loop_queue.output->velocity_z, sp);

  copter_io_->update_setpoint(sp);

#ifdef UAS_AT_UCLA_DEPLOYMENT
  digitalWrite(
      kAlarmGPIOPin,
      ::src::control::loops::flight_loop_queue.output->alarm ? HIGH : LOW);

  int bomb_drop_signal =
      ::src::control::loops::flight_loop_queue.output->bomb_drop ? 1000 : 1600;

  set_servo_pulsewidth(pigpio_, 23, bomb_drop_signal);

  int gimbal_angle =
      1500 +
      ::src::control::loops::flight_loop_queue.output->gimbal_angle / 90 * 500;
  set_servo_pulsewidth(pigpio_, 24, gimbal_angle);
#endif

  if (::src::control::loops::flight_loop_queue.output->trigger_takeoff + 0.05 >
      current_time) {
    if (!did_takeoff_) {
      did_takeoff_ = true;
      copter_io_->Takeoff();
    }
  } else {
    did_takeoff_ = false;
  }

  if (::src::control::loops::flight_loop_queue.output->trigger_hold + 0.05 >
      current_time) {
    if (!did_hold_) {
      did_hold_ = true;
      copter_io_->Hold();
    }
  } else {
    did_hold_ = false;
  }

  if (::src::control::loops::flight_loop_queue.output->trigger_offboard + 0.05 >
      current_time) {
    if (!did_offboard_) {
      did_offboard_ = true;
      copter_io_->Offboard();
    }
  } else {
    did_offboard_ = false;
  }

  if (::src::control::loops::flight_loop_queue.output->trigger_rtl + 0.05 >
      current_time) {
    if (!did_rtl_) {
      did_rtl_ = true;
      copter_io_->ReturnToLaunch();
    }
  } else {
    did_rtl_ = false;
  }

  if (::src::control::loops::flight_loop_queue.output->trigger_land + 0.05 >
      current_time) {
    if (!did_land_) {
      did_land_ = true;
      copter_io_->Land();
    }
  } else {
    did_land_ = false;
  }

  if (::src::control::loops::flight_loop_queue.output->trigger_arm + 0.05 >
      current_time) {
    if (!did_arm_) {
      did_arm_ = true;
      copter_io_->Arm();
    }
  } else {
    did_arm_ = false;
  }

  if (::src::control::loops::flight_loop_queue.output->trigger_disarm + 0.05 >
      current_time) {
    if (!did_disarm_) {
      did_disarm_ = true;
      copter_io_->Disarm();
    }
  } else {
    did_disarm_ = false;
  }
}

void AutopilotOutputWriter::Stop() {
  // No recent output queue messages received, so land drone.
  copter_io_->Land();

#ifdef UAS_AT_UCLA_DEPLOYMENT
  // Don't leave the alarm on after quitting code.
  digitalWrite(kAlarmGPIOPin, LOW);
#endif
}

}  // namespace io
}  // namespace control
}  // namespace src
