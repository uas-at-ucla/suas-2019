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
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include "sio_client.h"
#include "sio_socket.h"

#include "lib/base64_tools/base64_tools.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/proto_comms/proto_comms.h"
#include "lib/serial_comms/serial_comms_bridge.h"
#include "lib/serial_device/serial_device.h"
#include "src/controls/io/io.h"
#include "src/controls/messages.pb.h"

#include "src/controls/ground_controls/timeline/ground2drone_visitor/ground2drone_visitor.h"
#include "src/controls/ground_controls/timeline/timeline_grammar.pb.h"

#include "src/controls/constants.h"

namespace src {
namespace controls {
namespace ground_controls {

void on_connect();
void on_fail();

class GroundControls {
 public:
  GroundControls(int argc, char **argv);
  void ReadRFD900();
//   void ReadUDP();

  void OnConnect();
  void OnFail();

  ~GroundControls();

  ::std::atomic<bool> running_;

 private:
  void SensorsReceived(const ::src::controls::Sensors sensors);
  void DroneProgramReceived(
      const ::src::controls::ground_controls::timeline::DroneProgram
          drone_program);
  void MissionStatusReceived(const ::std_msgs::String mission_status);
  void SendSensorsToServer(const ::src::controls::Sensors &sensors,
                           bool rfd900);

  void GimbalSetpoint(const ::std_msgs::Float32 gimbal_setpoint);
  void DeploymentMotorSetpoint(const ::std_msgs::Float32 deployment_motor_setpoint);
  void LatchSetpoint(const ::std_msgs::Bool latch_setpoint);
  void HotwireSetpoint(const ::std_msgs::Bool hotwire_setpoint);

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber sensors_subscriber_;
  ::ros::Subscriber drone_program_subscriber_;
  ::ros::Subscriber mission_status_subscriber_;
  ::ros::Publisher drone_program_publisher_;
  ::ros::Publisher mission_status_publisher_;

  ::ros::Publisher gimbal_publisher_;
  ::ros::Publisher deployment_motor_publisher_;
  ::ros::Publisher latch_publisher_;
  ::ros::Publisher hotwire_publisher_;
  ::ros::Subscriber gimbal_subscriber_;
  ::ros::Subscriber deployment_motor_subscriber_;
  ::ros::Subscriber latch_subscriber_;
  ::ros::Subscriber hotwire_subscriber_;

//   ::lib::proto_comms::ProtoReceiver<::src::controls::UasMessage>
//       udp_connection_;
  ::lib::serial_device::SerialDevice<::src::controls::UasMessage>
      rfd900_connection_;

  // ::ros::Rate loop(50);
  ::sio::client client_;

  ::lib::phased_loop::PhasedLoop phased_loop_;

  ::src::controls::ground_controls::timeline::ground2drone_visitor::
      Ground2DroneVisitor ground2drone_visitor_;

  ::src::controls::ground_controls::timeline::DroneProgram drone_program_;
  bool drone_program_success_;
};

} // namespace ground_controls
} // namespace controls
} // namespace src
