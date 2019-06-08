#pragma once

#include <ros/ros.h>

#include "lib/proto_comms/proto_comms.h"
#include "lib/serial_device/serial_device.h"
#include "src/controls/ground_controls/ground_controls.h"
#include "src/controls/io/io.h"
#include "src/controls/messages.pb.h"

#include "src/controls/constants.h"

namespace src {
namespace controls {
namespace ground_communicator {

class GroundCommunicator {
 public:
  GroundCommunicator();

  void SensorsReceived(const ::src::controls::Sensors sensors);
  // void DroneProgramReceived(
  //     const ::src::controls::ground_controls::timeline::DroneProgram
  //         drone_program);

 private:
  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber sensors_subscriber_;
  // ::ros::Subscriber drone_program_subscriber_;

  ::lib::proto_comms::ProtoSender<::src::controls::UasMessage> proto_sender_;

  ::lib::serial_device::SerialDevice<::src::controls::UasMessage> rfd900_;
};

} // namespace ground_communicator
} // namespace controls
} // namespace src
