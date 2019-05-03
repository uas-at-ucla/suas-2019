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

#include <mavros_msgs/State.h>
#include <ros/console.h>
#include <ros/ros.h>

#include "sio_client.h"
#include "sio_socket.h"
#include "zmq.hpp"

#include "lib/base64_tools/base64_tools.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/proto_comms/proto_comms.h"
#include "lib/serial_device/serial_device.h"
#include "lib/serial_comms/serial_comms_bridge.h"
#include "src/controls/loops/flight_loop.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace ground_controls {
namespace {
static const int kRosMessageQueueSize = 1;
static const ::std::string kRosStateTopic = "/mavros/state";
// static const ::std::string kRosSensorsTopic = "/mavros/senors"; //TODO
} // namespace

void on_connect();
void on_fail();

class GroundControls {
 public:
  GroundControls();
  void Run();
  void RunIteration();

  void OnConnect();
  void OnFail();

  void ConnectToGround();
  ~GroundControls();

 private:
  void StateReceived(const ::mavros_msgs::State state);

  ::lib::proto_comms::ProtoReceiver<::src::controls::UasMessage> udp_connection_;
  ::lib::serial_device::SerialDevice<::src::controls::UasMessage> rfd900_connection_;

  ::sio::client client_;

  ::lib::phased_loop::PhasedLoop phased_loop_;
  ::std::atomic<bool> running_;
};

} // namespace ground_controls
} // namespace controls
} // namespace src
