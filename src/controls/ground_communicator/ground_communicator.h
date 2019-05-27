#pragma once

#include <ros/ros.h>

#include "lib/proto_comms/proto_comms.h"
#include "lib/serial_device/serial_device.h"
#include "src/controls/io/io.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace ground_communicator {

class GroundCommunicator {
 public:
  GroundCommunicator();

  void SensorsReceived(const ::src::controls::Sensors sensors);

 private:
  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber sensors_subscriber_;

  ::lib::proto_comms::ProtoSender<::src::controls::UasMessage> proto_sender_;

  ::lib::serial_device::SerialDevice<::src::controls::UasMessage> rfd900_;
};

} // namespace ground_communicator
} // namespace controls
} // namespace src
