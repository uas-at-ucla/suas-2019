#include "ros_to_proto.h"

namespace src {
namespace controls {
namespace io {
namespace ros_to_proto {

RosToProto::RosToProto() :
    global_position_subscriber_(ros_node_handle_.subscribe(
        kRosGlobalPositionTopic, kRosMessageQueueSize,
        &RosToProto::GlobalPositionReceived, this)),
    altitude_subscriber_(
        ros_node_handle_.subscribe(kRosAltitudeTopic, kRosMessageQueueSize,
                                   &RosToProto::AltitudeReceived, this)),
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
  ros_topic_last_received_times_[kRosAltitudeTopic] =
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

  /*
     In the position_covariance 3x3 matrix
     (http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html), the square
     roots of the diagonals should be the x, y, and z (or East, North, and Up)
     uncertaintes, in that order. The rest of the matrix is not set. This can be
     confirmed by the mavros source code
     (https://github.com/mavlink/mavros/blob/a7ef4fc0ec153307cbce3c98a998956c6296b954/mavros/src/plugins/global_position.cpp#L178).
     In our case, mavros should use h_acc and v_acc from GPS_RAW_INT
     (https://mavlink.io/en/messages/common.html#GPS_RAW_INT), but notice how it
     can also use raw_gps.eph - Don't be fooled! This eph is a misnomer
     (https://github.com/mavlink/mavlink/issues/1063). This even confused a
     mavros developer who labeled the units of eph as meters in a diagnostic
     message!
     (https://github.com/mavlink/mavros/blob/a7ef4fc0ec153307cbce3c98a998956c6296b954/mavros/src/plugins/global_position.cpp#L446)
     Despite this, the calculations in the code are still correct, as
     raw_gps.eph is HDOP scaled by 100, as set by PX4
     (https://github.com/PX4/Firmware/blob/c95394f57f0a771ed28d1941daf6b52ee3a70b5b/src/modules/mavlink/mavlink_messages.cpp#L1395)
     The calculation in mavros looks like it's based on this:
     https://en.wikipedia.org/wiki/Error_analysis_for_the_Global_Positioning_System
     At this point, I went a little further down the rabbit hole, all the way to
     GPS module protocols. HDOP does not seem to be defined consistently. Here
     it's unitless, as it should be:
     https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual1.pdf But
     here it's in meters! http://freenmea.net/docs.
     tl;dr, think twice before using anything that says eph, epv, HDOP, or VDOP.
  */
  sensors_.set_gps_eph(sqrt(global_position.position_covariance[0]));
  sensors_.set_gps_epv(sqrt(global_position.position_covariance[8]));
}

void RosToProto::AltitudeReceived(const ::mavros_msgs::Altitude altitude) {
  GotRosMessage(kRosAltitudeTopic);

  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
  sensors_.set_altitude(altitude.amsl);
  sensors_.set_relative_altitude(altitude.relative);
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
  (void)diagnostic_array;

  GotRosMessage(kRosDiagnosticsTopic);

  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
  for (size_t i = 0; i < diagnostic_array.status.size(); i++) {
    ::diagnostic_msgs::DiagnosticStatus &status = diagnostic_array.status[i];
    if (status.name == "mavros: GPS") {
      for (size_t j = 0; j < status.values.size(); j++) {
        ::diagnostic_msgs::KeyValue &key_value = status.values[j];
        if (key_value.key == "Satellites visible") {
          sensors_.set_gps_satellite_count(::std::stoi(key_value.value));
        }
      }
    }
  }
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

  double roll, pitch, yaw;
  toEulerAngle(imu_data.orientation, roll, pitch, yaw);
  sensors_.set_roll(roll);
  sensors_.set_pitch(pitch);
  sensors_.set_yaw(yaw);
}

void RosToProto::BatteryStateReceived(
    ::sensor_msgs::BatteryState battery_state) {

  GotRosMessage(kRosBatteryStateTopic);

  ::std::lock_guard<::std::mutex> lock(sensors_mutex_);
  sensors_.set_battery_voltage(battery_state.voltage);
  sensors_.set_battery_current(battery_state.current);
}

bool RosToProto::SensorsValid() {
  if (!sensors_.IsInitialized()) { // all required fields are filled
    return false;
  }

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

// from
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void toEulerAngle(const ::geometry_msgs::Quaternion &q, double &roll,
                  double &pitch, double &yaw) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = atan2(siny_cosp, cosy_cosp);
}

} // namespace ros_to_proto
} // namespace io
} // namespace controls
} // namespace src
