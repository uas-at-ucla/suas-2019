#pragma once

#include <functional>

#include <common/mavlink.h>
#include <ros/ros.h>

#include "src/controls/messages.pb.h"
#include "src/controls/io/autopilot_interface/ros_publisher/ros_publisher.h"
#include "lib/mavconn_udp/interface.h"

namespace src {
namespace controls {
namespace io {
namespace autopilot_interface {
namespace ros_publisher {

class RosPublisher {
 public:
  RosPublisher();

  void WriteMessage(const mavlink_message_t *msg,
                    const ::mavconn::Framing framing);

 private:
  ::ros::NodeHandle ros_node_handle_;

  ::ros::Publisher local_position_ned_publisher_;
};

} // namespace ros_publisher
} // namespace autopilot_interface
} // namespace io
} // namespace controls
} // namespace src
