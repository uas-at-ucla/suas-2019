#pragma once

#include <functional>

#include <common/mavlink.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "lib/mavconn_udp/interface.h"
#include "src/controls/io/autopilot_interface/ros_publisher/ros_publisher.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace io {
namespace autopilot_interface {
namespace ros_publisher {
namespace {
static const int kRosQueueSize = 1;
} // namespace

class RosPublisher {
 public:
  RosPublisher();

  void WriteMessage(const mavlink_message_t *msg,
                    const ::mavconn::Framing framing);

 private:
  ::ros::NodeHandle ros_node_handle_;

  ::ros::Publisher heartbeat_publisher_;
  ::ros::Publisher sys_status_publisher_;
  ::ros::Publisher battery_status_publisher_;
  ::ros::Publisher radio_status_publisher_;
  ::ros::Publisher local_position_ned_publisher_;
  ::ros::Publisher global_position_int_publisher_;
  ::ros::Publisher gps_raw_int_publisher_;
  ::ros::Publisher highres_imu_publisher_;
  ::ros::Publisher attitude_publisher_;
  ::ros::Publisher vfr_hud_publisher_;
  ::ros::Publisher actuator_control_target_publisher_;
  ::ros::Publisher altitude_publisher_;
};

} // namespace ros_publisher
} // namespace autopilot_interface
} // namespace io
} // namespace controls
} // namespace src
