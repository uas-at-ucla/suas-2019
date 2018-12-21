#include "io_sim.h"
#include <math.h>

namespace src {
namespace controls {
namespace io {

IO *io_quit;
void quit_handler(int sig) {
  (void)sig;
  io_quit->Quit();
}

IO::IO() : autopilot_sensor_reader_(), autopilot_output_writer_() {
  io_quit = this;
  signal(SIGINT, quit_handler);
}

void IO::Run() {
  pos_info.latitude = 0;
  pos_info.longitude = 0;
  pos_info.altitude = 100;
  pos_info.velocity_x = 0;
  pos_info.velocity_y = 0;
  pos_info.velocity_z = 0;

  ::std::thread autopilot_sensor_reader_thread(
      ::std::ref(autopilot_sensor_reader_));
  ::std::thread autopilot_output_writer_thread(
      ::std::ref(autopilot_output_writer_));

  autopilot_sensor_reader_thread.join();
  autopilot_output_writer_thread.join();
}

void IO::Quit() {
  run_ = false;
  autopilot_sensor_reader_.Quit();
  autopilot_output_writer_.Quit();
}

AutopilotSensorReader::AutopilotSensorReader() :
    last_gps_(-::std::numeric_limits<double>::infinity()),
    sensors_sender_("ipc:///tmp/uasatucla_sensors.ipc") {}

void AutopilotSensorReader::RunIteration() {
  std::lock_guard<std::mutex> lock(pos_info_mutex);

  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  ::src::controls::Sensors *sensors = new ::src::controls::Sensors();

  // GPS data.
  {
    sensors->set_latitude(pos_info.latitude);
    sensors->set_longitude(pos_info.longitude);
    sensors->set_altitude(pos_info.altitude);

    sensors->set_relative_altitude(pos_info.altitude);

    sensors->set_velocity_x(pos_info.velocity_x);
    sensors->set_velocity_y(pos_info.velocity_y);
    sensors->set_velocity_z(pos_info.velocity_z);
  }

  sensors->set_heading(0);

  // GPS raw data.
  {
    sensors->set_gps_satellite_count(0);
    sensors->set_gps_eph(0);
    sensors->set_gps_epv(0);
    sensors->set_gps_ground_speed(
        sqrt(pow(pos_info.velocity_x, 2) + pow(pos_info.velocity_y, 2)));
  }

  // IMU data.
  {
    sensors->set_accelerometer_x(0);
    sensors->set_accelerometer_y(0);
    sensors->set_accelerometer_z(0);

    sensors->set_gyro_x(0);
    sensors->set_gyro_y(0);
    sensors->set_gyro_z(0);

    sensors->set_absolute_pressure(0);
    sensors->set_relative_pressure(0);
    sensors->set_pressure_altitude(0);
    sensors->set_temperature(0);
  }

  sensors->set_last_gps(0);

  // Battery data.
  {
    sensors->set_battery_voltage(0);
    sensors->set_battery_current(0);
  }

  // Armed?
  {
    sensors->set_armed(true);

    AutopilotState autopilot_state = UNKNOWN;
    // Not testing autopilot functionality for now (land, takeoff, etc.), always
    // offboard
    autopilot_state = OFFBOARD;

    sensors->set_autopilot_state(autopilot_state);
  }

  // Serialize sensor protobuf and send it over ZMQ.
  UasMessage message = ::src::controls::UasMessage();
  message.set_time(current_time);
  message.set_allocated_sensors(sensors);
  sensors_sender_.Send(message);
}

AutopilotOutputWriter::AutopilotOutputWriter() :
    output_receiver_("ipc:///tmp/uasatucla_output.ipc", 5),
    takeoff_trigger_(kTriggerSignalTolerance),
    hold_trigger_(kTriggerSignalTolerance),
    offboard_trigger_(kTriggerSignalTolerance),
    rtl_trigger_(kTriggerSignalTolerance),
    land_trigger_(kTriggerSignalTolerance),
    arm_trigger_(kTriggerSignalTolerance),
    disarm_trigger_(kTriggerSignalTolerance) {}

void AutopilotOutputWriter::RunIteration() {
  std::lock_guard<std::mutex> lock(pos_info_mutex);

  if (output_receiver_.HasMessages()) {
    ::src::controls::Output output;
    output = output_receiver_.GetLatest();
    pos_info.velocity_x = output.velocity_x();
    pos_info.velocity_y = output.velocity_y();
    pos_info.velocity_z = output.velocity_z();
    // autopilot_interface::set_yaw(output.yaw_setpoint(), sp);
  }

  // TODO: integrate velocity to update position

  // if (takeoff_trigger_.Process(output.trigger_takeoff())) {
  //   copter_io_->Takeoff();
  // }

  // if (hold_trigger_.Process(output.trigger_hold())) {
  //   copter_io_->Hold();
  // }

  // if (offboard_trigger_.Process(output.trigger_offboard())) {
  //   copter_io_->Offboard();
  // }

  // if (offboard_trigger_.Process(output.trigger_rtl())) {
  //   copter_io_->ReturnToLaunch();
  // }

  // if (land_trigger_.Process(output.trigger_land())) {
  //   copter_io_->Land();
  // }

  // if (arm_trigger_.Process(output.trigger_arm())) {
  //   copter_io_->Arm();
  // }

  // if (disarm_trigger_.Process(output.trigger_disarm())) {
  //   copter_io_->Disarm();
  // }
}

void AutopilotOutputWriter::Stop() {}

} // namespace io
} // namespace controls
} // namespace src
