#include "ros_to_proto.h"

namespace src {
namespace controls {
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
                                   &RosToProto::StateReceived, this)) {}

void RosToProto::GlobalPositionReceived(
    const ::sensor_msgs::NavSatFix global_position) {

  sensors_.set_latitude(global_position.latitude);
  sensors_.set_longitude(global_position.longitude);
  sensors_.set_altitude(global_position.altitude);
}

void RosToProto::RelativeAltitudeReceived(
    const ::std_msgs::Float64 relative_altitude) {
  sensors_.set_relative_altitude(relative_altitude.data);
}

void RosToProto::CompassHeadingReceived(
    const ::std_msgs::Float64 compass_heading) {
  sensors_.set_heading(compass_heading.data);
}

void RosToProto::VelocityReceived(
    const ::geometry_msgs::TwistStamped velocity) {
  sensors_.set_velocity_x(velocity.twist.linear.x);
  sensors_.set_velocity_y(velocity.twist.linear.y);
  sensors_.set_velocity_z(velocity.twist.linear.z);
}

void RosToProto::VfrHudReceived(const ::mavros_msgs::VFR_HUD vfr_hud) {
  sensors_.set_gps_ground_speed(vfr_hud.groundspeed);
}

void RosToProto::DiagnosticsReceived(
    ::diagnostic_msgs::DiagnosticArray diagnostic_array) {
  sensors_.set_gps_satellite_count(::std::stoi(diagnostic_array.status[1].values[0].value));
  sensors_.set_gps_eph(::std::stod(diagnostic_array.status[1].values[2].value));
  sensors_.set_gps_epv(::std::stod(diagnostic_array.status[1].values[3].value));
}

void RosToProto::ImuDataReceived(::sensor_msgs::Imu imu_data) {
  sensors_.set_accelerometer_x(imu_data.linear_acceleration.x);
  sensors_.set_accelerometer_y(imu_data.linear_acceleration.y);
  sensors_.set_accelerometer_z(imu_data.linear_acceleration.z);

  sensors_.set_gyro_x(imu_data.angular_velocity.x);
  sensors_.set_gyro_y(imu_data.angular_velocity.y);
  sensors_.set_gyro_z(imu_data.angular_velocity.z);
}

void RosToProto::BatteryStateReceived(
    ::sensor_msgs::BatteryState battery_state) {
  sensors_.set_battery_voltage(battery_state.voltage);
  sensors_.set_battery_current(battery_state.current);
}

void RosToProto::StateReceived(::mavros_msgs::State state) {
  sensors_.set_armed(state.armed);
  sensors_.set_autopilot_state(state.mode);
}

} // namespace ros_to_proto
} // namespace controls
} // namespace src
