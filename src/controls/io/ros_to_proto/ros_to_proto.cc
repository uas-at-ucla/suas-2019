#include "ros_to_proto.h"

namespace src {
namespace controls {
namespace io {
namespace ros_to_proto {

RosToProto::RosToProto() :
    global_position_subscriber_(ros_node_handle_.subscribe(
        kRosGlobalPositionTopic, kRosMessageQueueSize,
        &RosToProto::GlobalPositionReceived, this)),
    relative_altitude_subscriber_(ros_node_handle_.subscribe(
        kRosRelativeAltitudeTopic, kRosMessageQueueSize,
        &RosToProto::RelativeAltitudeReceived, this)),
    compass_heading_subscriber_(ros_node_handle_.subscribe(
        kRosCompassHeadingTopic, kRosMessageQueueSize,
        &RosToProto::CompassHeadingReceived, this)),
    velocity_subscriber_(
        ros_node_handle_.subscribe(kRosVelocityTopic, kRosMessageQueueSize,
                                   &RosToProto::VelocityReceived, this)),
    vfr_hud_subscriber_(
        ros_node_handle_.subscribe(kRosVfrHudTopic, kRosMessageQueueSize,
                                   &RosToProto::VfrHudReceived, this)),
    diagnostics_subscriber_(
        ros_node_handle_.subscribe(kRosDiagnosticsTopic, kRosMessageQueueSize,
                                   &RosToProto::DiagnosticsReceived, this)),
    imu_subscriber_(
        ros_node_handle_.subscribe(kRosImuDataTopic, kRosMessageQueueSize,
                                   &RosToProto::ImuDataReceived, this)),
    battery_state_subscriber_(
        ros_node_handle_.subscribe(kRosBatteryStateTopic, kRosMessageQueueSize,
                                   &RosToProto::BatteryStateReceived, this)),
    state_subscriber_(
        ros_node_handle_.subscribe(kRosStateTopic, kRosMessageQueueSize,
                                   &RosToProto::StateReceived, this)),
    arming_service_(ros_node_handle_.serviceClient<mavros_msgs::CommandBool>(
        kRosArmingService)) {

  // Initialize all last received times for the ros topics to NaN.
  ros_topic_last_received_times_[kRosGlobalPositionTopic] =
      std::numeric_limits<double>::quiet_NaN();
  ros_topic_last_received_times_[kRosRelativeAltitudeTopic] =
      std::numeric_limits<double>::quiet_NaN();
  ros_topic_last_received_times_[kRosCompassHeadingTopic] =
      std::numeric_limits<double>::quiet_NaN();
  ros_topic_last_received_times_[kRosVelocityTopic] =
      std::numeric_limits<double>::quiet_NaN();
  ros_topic_last_received_times_[kRosVfrHudTopic] =
      std::numeric_limits<double>::quiet_NaN();
  ros_topic_last_received_times_[kRosDiagnosticsTopic] =
      std::numeric_limits<double>::quiet_NaN();
  ros_topic_last_received_times_[kRosImuDataTopic] =
      std::numeric_limits<double>::quiet_NaN();
  ros_topic_last_received_times_[kRosBatteryStateTopic] =
      std::numeric_limits<double>::quiet_NaN();
  ros_topic_last_received_times_[kRosStateTopic] =
      std::numeric_limits<double>::quiet_NaN();
}

Sensors RosToProto::GetSensors() {
  Sensors sensors_copy;

  // Grab lock to prevent ROS from modifying the shared Sensors object while a
  // copy is performed.
  {
    ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
    sensors_copy.CopyFrom(sensors_);
  }

  return sensors_copy;
}

void RosToProto::SendOutput(Output output) {
  mavros_msgs::CommandBool arming_cmd;
  arming_cmd.request.value = true;
  arming_service_.call(arming_cmd);
  ::std::cout << "ARM!\n";

  if (output.send_offboard()) {
  }
}

void RosToProto::GlobalPositionReceived(
    const ::sensor_msgs::NavSatFix global_position) {

  GotRosMessage(kRosGlobalPositionTopic);

  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
  sensors_.set_latitude(global_position.latitude);
  sensors_.set_longitude(global_position.longitude);
  sensors_.set_altitude(global_position.altitude);
}

void RosToProto::RelativeAltitudeReceived(
    const ::std_msgs::Float64 relative_altitude) {

  GotRosMessage(kRosRelativeAltitudeTopic);

  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
  sensors_.set_relative_altitude(relative_altitude.data);
  sensors_mutex_.unlock();
}

void RosToProto::CompassHeadingReceived(
    const ::std_msgs::Float64 compass_heading) {

  GotRosMessage(kRosCompassHeadingTopic);

  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
  sensors_.set_heading(compass_heading.data);
  sensors_mutex_.unlock();
}

void RosToProto::VelocityReceived(
    const ::geometry_msgs::TwistStamped velocity) {

  GotRosMessage(kRosVelocityTopic);

  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
  sensors_.set_velocity_x(velocity.twist.linear.x);
  sensors_.set_velocity_y(velocity.twist.linear.y);
  sensors_.set_velocity_z(velocity.twist.linear.z);
}

void RosToProto::VfrHudReceived(const ::mavros_msgs::VFR_HUD vfr_hud) {
  GotRosMessage(kRosVfrHudTopic);

  sensors_.set_gps_ground_speed(vfr_hud.groundspeed);
}

void RosToProto::DiagnosticsReceived(
    ::diagnostic_msgs::DiagnosticArray diagnostic_array) {

  GotRosMessage(kRosDiagnosticsTopic);

  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
  sensors_.set_gps_satellite_count(
      ::std::stoi(diagnostic_array.status[1].values[0].value));
  sensors_.set_gps_eph(::std::stod(diagnostic_array.status[1].values[2].value));
  sensors_.set_gps_epv(::std::stod(diagnostic_array.status[1].values[3].value));
}

void RosToProto::ImuDataReceived(::sensor_msgs::Imu imu_data) {
  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);

  GotRosMessage(kRosImuDataTopic);

  sensors_.set_accelerometer_x(imu_data.linear_acceleration.x);
  sensors_.set_accelerometer_y(imu_data.linear_acceleration.y);
  sensors_.set_accelerometer_z(imu_data.linear_acceleration.z);

  sensors_.set_gyro_x(imu_data.angular_velocity.x);
  sensors_.set_gyro_y(imu_data.angular_velocity.y);
  sensors_.set_gyro_z(imu_data.angular_velocity.z);
}

void RosToProto::BatteryStateReceived(
    ::sensor_msgs::BatteryState battery_state) {

  GotRosMessage(kRosBatteryStateTopic);

  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
  sensors_.set_battery_voltage(battery_state.voltage);
  sensors_.set_battery_current(battery_state.current);
}

bool RosToProto::SensorsValid() {
  double oldest_valid_packet_time =
      ::lib::phased_loop::GetCurrentTime() - kRosReceiveTolerance;

  for (::std::map<::std::string, double>::iterator it =
           ros_topic_last_received_times_.begin();
       it != ros_topic_last_received_times_.end(); it++) {
    if (it->second < oldest_valid_packet_time) {
      return false;
    }
  }

  return true;
}

void RosToProto::StateReceived(::mavros_msgs::State state) {
  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);

  GotRosMessage(kRosStateTopic);

  sensors_.set_armed(state.armed);
  sensors_.set_autopilot_state(state.mode);
}

void RosToProto::GotRosMessage(::std::string ros_topic) {
  ros_topic_last_received_times_[ros_topic] =
      ::lib::phased_loop::GetCurrentTime();
}

} // namespace ros_to_proto
} // namespace io
} // namespace controls
} // namespace src
