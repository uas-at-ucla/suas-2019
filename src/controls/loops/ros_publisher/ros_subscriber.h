#pragma once

#include <functional>

#include <common/mavlink.h>
#include <ros/ros.h>

#include "lib/mavconn_udp/interface.h"
#include "src/controls/io/autopilot_interface/ros_subscriber/ros_subscriber.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace io {
namespace autopilot_interface {
namespace ros_subscriber {

class RosSubscriber {
 public:
  RosSubscriber();

  void ReadMessage(const ::src::controls::Message &message);

 private:
  ::ros::NodeHandle ros_node_handle_;

  ::ros::Subscriber heartbeat_subscriber_;
  ::ros::Subscriber sys_status_subscriber_;
  ::ros::Subscriber battery_status_subscriber_;
  ::ros::Subscriber radio_status_subscriber_;
  ::ros::Subscriber local_position_ned_subscriber_;
  ::ros::Subscriber global_position_int_subscriber_;
  ::ros::Subscriber gps_raw_int_subscriber_;
  ::ros::Subscriber highres_imu_subscriber_;
  ::ros::Subscriber attitude_subscriber_;
  ::ros::Subscriber vfr_hud_subscriber_;
  ::ros::Subscriber actuator_control_target_subscriber_;
};

} // namespace ros_subscriber
} // namespace autopilot_interface
} // namespace io
} // namespace controls
} // namespace src
