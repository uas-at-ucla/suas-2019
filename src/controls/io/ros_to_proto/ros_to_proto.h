#pragma once

#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <string>

#include <ros/console.h>
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <google/protobuf/text_format.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

#include "lib/phased_loop/phased_loop.h"
#include "src/controls/constants.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace io {
namespace ros_to_proto {

class RosToProto {
 public:
  RosToProto();

  Sensors GetSensors();
  bool SensorsValid();
  void SetRunUasMission(bool run);
  bool OutputValid();
  ::src::controls::Output GetOutput();

 private:
  void GlobalPositionReceived(const ::sensor_msgs::NavSatFix global_position);
  void AltitudeReceived(const ::mavros_msgs::Altitude altitude);
  void CompassHeadingReceived(const ::std_msgs::Float64 compass_heading);
  void VelocityReceived(const ::geometry_msgs::TwistStamped velocity);
  void VfrHudReceived(::mavros_msgs::VFR_HUD vfr_hud);
  void DiagnosticsReceived(::diagnostic_msgs::DiagnosticArray diagnostic_array);
  void ImuDataReceived(::sensor_msgs::Imu imu_data);
  void BatteryStateReceived(::sensor_msgs::BatteryState battery_state);
  void StateReceived(::mavros_msgs::State state);
  void HomePositionReceived(const ::mavros_msgs::HomePosition home_position);

  void GotRosMessage(::std::string ros_topic);

  void OutputReceived(::src::controls::Output output);

  Sensors sensors_;
  ::std::mutex sensors_mutex_;

  Output output_;
  ::std::mutex output_mutex_;

  ::std::map<::std::string, double> ros_topic_last_received_times_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber global_position_subscriber_;
  ::ros::Subscriber altitude_subscriber_;
  ::ros::Subscriber compass_heading_subscriber_;
  ::ros::Subscriber velocity_subscriber_;
  ::ros::Subscriber vfr_hud_subscriber_;
  ::ros::Subscriber diagnostics_subscriber_;
  ::ros::Subscriber imu_subscriber_;
  ::ros::Subscriber battery_state_subscriber_;
  ::ros::Subscriber state_subscriber_;
  ::ros::Subscriber home_position_subscriber_;
  ::ros::Subscriber output_subscriber_;
  ::ros::ServiceClient arming_service_;

  double last_output_;
};

void toEulerAngle(const ::geometry_msgs::Quaternion &q, double &roll,
                  double &pitch, double &yaw);

} // namespace ros_to_proto
} // namespace io
} // namespace controls
} // namespace src
