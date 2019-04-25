#pragma once

#include <ros/ros.h>

#include "sio_client.h"
#include "sio_socket.h"

#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace ground_communicator {
namespace {
static const int kRosMessageQueueSize = 1;

static const ::std::string kRosUgvSensorsTopic = "/uasatucla/proto/uav_sensors";
} // namespace

class GroundCommunicator {
 public:
  GroundCommunicator();

  void UgvSensorsReceived(const ::src::controls::UgvSensors ugv_sensors);
  void DroneSensorsReceived(const ::src::controls::Sensors drone_sensors);

 private:
  ::ros::NodeHandle ros_node_handle_;
  ::ros::Publisher ground_controls_publisher_;
};

} // namespace ground_server
} // namespace controls
} // namespace src
