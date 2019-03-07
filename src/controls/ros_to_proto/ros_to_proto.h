#pragma once

#include <string>

#include <ros/console.h>
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <google/protobuf/text_format.h>

#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace ros_to_proto {
namespace {
static const int kRosMessageQueueSize = 1;
static const ::std::string kRosGlobalPositionTopic =
    "/mavros/global_position/global";
static const ::std::string kRosRelativeAltitudeTopic =
    "/mavros/global_position/rel_alt";
static const ::std::string kRosCompassHeadingTopic =
    "/mavros/global_position/compass_hdg";
static const ::std::string kRosVelocityTopic =
    "/mavros/local_position/velocity";
static const ::std::string kRosVfrHudTopic = "/mavros/vfr_hud";
static const ::std::string kRosDiagnosticsTopic = "/diagnostics";
static const ::std::string kRosImuDataTopic = "/mavros/imu/data";
static const ::std::string kRosBatteryStateTopic = "/mavros/battery";
static const ::std::string kRosStateTopic = "/mavros/state";
} // namespace

// RosToProto base class
class RosToProto {
 public:
  RosToProto();

 private:
  void GlobalPositionReceived(const ::sensor_msgs::NavSatFix global_position);
  void RelativeAltitudeReceived(const ::std_msgs::Float64 relative_altitude);
  void CompassHeadingReceived(const ::std_msgs::Float64 compass_heading);
  void VelocityReceived(const ::geometry_msgs::TwistStamped velocity);
  void VfrHudReceived(::mavros_msgs::VFR_HUD vfr_hud);
  void DiagnosticsReceived(::diagnostic_msgs::DiagnosticArray diagnostic_array);
  void ImuDataReceived(::sensor_msgs::Imu imu_data);
  void BatteryStateReceived(::sensor_msgs::BatteryState battery_state);
  void StateReceived(::mavros_msgs::State state);

  Sensors sensors_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber global_position_subscriber_;
  ::ros::Subscriber relative_altitude_subscriber_;
  ::ros::Subscriber compass_heading_subscriber_;
  ::ros::Subscriber velocity_subscriber_;
  ::ros::Subscriber vfr_hud_subscriber_;
  ::ros::Subscriber diagnostics_subscriber_;
  ::ros::Subscriber imu_subscriber_;
  ::ros::Subscriber battery_state_subscriber_;
  ::ros::Subscriber state_subscriber_;
};

} // namespace ros_to_proto
} // namespace controls
} // namespace src
