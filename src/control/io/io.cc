#include "io.h"

namespace spinny {
namespace control {
namespace io {

IO::IO() : copter_io_("/tmp/virtualcom0", 115200) {}

void IO::Run() { RunAutopilotIO(); }

void IO::RunAutopilotIO() {
  copter_io_.start();

  for (int i = 0; i < 25 || true; i++) {
    auto flight_loop_sensors_message =
        ::spinny::control::loops::flight_loop_queue.sensors.MakeMessage();

    // GPS data.
    mavlink_global_position_int_t gps =
        copter_io_.current_messages.global_position_int;

    flight_loop_sensors_message->latitude = static_cast<float>(gps.lat) / 1e7;
    flight_loop_sensors_message->longitude = static_cast<float>(gps.lon) / 1e7;
    flight_loop_sensors_message->altitude = static_cast<float>(gps.alt) / 1e3;

    flight_loop_sensors_message->relative_altitude =
        static_cast<float>(gps.relative_alt) / 1e3;

    flight_loop_sensors_message->velocity_x = static_cast<float>(gps.vx) / 1e2;
    flight_loop_sensors_message->velocity_y = static_cast<float>(gps.vy) / 1e2;
    flight_loop_sensors_message->velocity_z = static_cast<float>(gps.vz) / 1e2;

    // IMU data.
    mavlink_highres_imu_t imu = copter_io_.current_messages.highres_imu;

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
    mavlink_sys_status_t sys_status = copter_io_.current_messages.sys_status;
    flight_loop_sensors_message->battery_voltage =
        static_cast<float>(sys_status.voltage_battery) / 1e3;
    flight_loop_sensors_message->battery_current =
        static_cast<float>(sys_status.current_battery) / 1e3;

    flight_loop_sensors_message.Send();

    usleep(1e6 / 40);
  }

  copter_io_.stop();
}

}  // namespace io
}  // namespace control
}  // namespace spinny
