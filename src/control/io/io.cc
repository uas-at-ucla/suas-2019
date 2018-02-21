#include "io.h"

#include <iostream>

namespace spinny {
namespace control {
namespace io {

autopilot_interface::AutopilotInterface *copter_io_quit;
IO *io_quit;
void quit_handler(int sig) {
  printf("\n\nTERMINATING AT USER REQUEST\n\n");

  io_quit->Quit();

  try {
    copter_io_quit->handle_quit(sig);
  } catch (int error) {
  }

  exit(0);
}

IO::IO()
//  : copter_io_("/dev/ttyS0", 921600) {
    : copter_io_("/tmp/virtualcom0", 921600),
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
    : copter_io_(copter_io) {

  last_timestamps_.reset_timestamps();
}

void AutopilotSensorReader::RunIteration() {
  // TODO(comran): Fix partial queue writes.
  autopilot_interface::TimeStamps current_timestamps =
      copter_io_->current_messages.time_stamps;

  if (!(current_timestamps.global_position_int -
        last_timestamps_.global_position_int)) {
    return;
  }

  last_timestamps_ = current_timestamps;

  auto flight_loop_sensors_message =
      ::spinny::control::loops::flight_loop_queue.sensors.MakeMessage();

  // GPS data.
  mavlink_global_position_int_t gps =
      copter_io_->current_messages.global_position_int;

  flight_loop_sensors_message->latitude = static_cast<double>(gps.lat) / 1e7;
  flight_loop_sensors_message->longitude = static_cast<double>(gps.lon) / 1e7;
  flight_loop_sensors_message->altitude = static_cast<float>(gps.alt) / 1e3;

  flight_loop_sensors_message->relative_altitude =
      static_cast<float>(gps.relative_alt) / 1e3;

  flight_loop_sensors_message->velocity_x = static_cast<float>(gps.vx) / 1e2;
  flight_loop_sensors_message->velocity_y = static_cast<float>(gps.vy) / 1e2;
  flight_loop_sensors_message->velocity_z = static_cast<float>(gps.vz) / 1e2;

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

  // Battery data.
  mavlink_sys_status_t sys_status = copter_io_->current_messages.sys_status;
  flight_loop_sensors_message->battery_voltage =
      static_cast<float>(sys_status.voltage_battery) / 1e3;
  flight_loop_sensors_message->battery_current =
      static_cast<float>(sys_status.current_battery) / 1e3;

  // Armed?
  mavlink_heartbeat_t heartbeat = copter_io_->current_messages.heartbeat;
  flight_loop_sensors_message->armed = !!(heartbeat.base_mode >> 7);

  flight_loop_sensors_message.Send();
}

AutopilotOutputWriter::AutopilotOutputWriter(
    autopilot_interface::AutopilotInterface *copter_io)
    : copter_io_(copter_io) {}

void AutopilotOutputWriter::Read() {
  ::spinny::control::loops::flight_loop_queue.output.FetchAnother();
  ::spinny::control::loops::flight_loop_queue.sensors.FetchAnother();
}

void AutopilotOutputWriter::Write() {
  if (::spinny::control::loops::flight_loop_queue.output->velocity_control) {
    mavlink_set_position_target_local_ned_t sp;

    autopilot_interface::set_velocity(
        ::spinny::control::loops::flight_loop_queue.output->velocity_x,
        ::spinny::control::loops::flight_loop_queue.output->velocity_y,
        ::spinny::control::loops::flight_loop_queue.output->velocity_z, sp);

    copter_io_->update_setpoint(sp);
  }

  if (::spinny::control::loops::flight_loop_queue.output->arm) {
    copter_io_->Arm();
  }

  if (::spinny::control::loops::flight_loop_queue.output->takeoff) {
    copter_io_->Takeoff();
  }

  if (::spinny::control::loops::flight_loop_queue.output->velocity_control) {
    //TODO(comran): Check altitude is above normal (on restarts).
    copter_io_->Offboard();
  }

  if (::spinny::control::loops::flight_loop_queue.output->land) {
    copter_io_->Land();
  }

  if (::spinny::control::loops::flight_loop_queue.output->throttle_cut) {
    copter_io_->FlightTermination();
  }
}

void AutopilotOutputWriter::Stop() {
  // No recent output queue messages received, so land drone.
  copter_io_->Land();
}

}  // namespace io
}  // namespace control
}  // namespace spinny
