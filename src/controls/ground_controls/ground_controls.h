#pragma once

#include <atomic>
#include <bitset>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/console.h>
#include <ros/ros.h>

#include "sio_client.h"
#include "sio_socket.h"

#include "lib/base64_tools/base64_tools.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/proto_comms/proto_comms.h"
#include "lib/serial_comms/serial_comms_bridge.h"
#include "lib/serial_device/serial_device.h"
#include "src/controls/io/io.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace ground_controls {
namespace {
static const int kRosMessageQueueSize = 1;
} // namespace

void on_connect();
void on_fail();

class GroundControls {
 public:
  GroundControls();
  void ReadRFD900();

  void OnConnect();
  void OnFail();

  ~GroundControls();

 private:
  void SensorsReceived(const ::src::controls::Sensors sensors);

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber sensors_subscriber_;

  // ::lib::proto_comms::ProtoReceiver<::src::controls::UasMessage>
  // udp_connection_;
  ::lib::serial_device::SerialDevice<::src::controls::UasMessage>
      rfd900_connection_;

  // ::ros::Rate loop(50);
  ::sio::client client_;

  ::lib::phased_loop::PhasedLoop phased_loop_;
  ::std::atomic<bool> running_;
};

} // namespace ground_controls
} // namespace controls
} // namespace src
