#include "ground_controls.h"
#include "lib/serial_device/serial_device.h"
#include "src/controls/messages.pb.h"
#include <ros/ros.h>
#include <vector>

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_ground_controls");
  ::ros::start();

  // ::src::controls::ground_controls::GroundControls ground_controls;
  // ground_controls.Run();

  // Serial device test
  ::lib::serial_device::SerialDevice<::src::controls::UasMessage> rfd900(
      "/dev/ttyUSB0", B57600, 0);
  ::lib::proto_comms::ProtoReceiver<::src::controls::UasMessage> udp(
      "tcp://127.0.0.1:5555", 1);

  // ::ros::Rate loop(30);

  /*
  ::src::controls::UasMessage current_message;

  while (true) {
    if (!::ros::ok()) {
      break;
    }

    if (rfd900.GetLatestProto(current_message) &&
        current_message.has_sensors()) {
      ::std::cout << "Battery Voltage: "
                  << current_message.sensors().battery_voltage()
                  << " Latitude: " << current_message.sensors().latitude()
                  << " Longitude: " << current_message.sensors().longitude()
                  << " Altitude: " << current_message.sensors().altitude()
                  << ::std::endl;
    }

    ::ros::spinOnce();
    loop.sleep();
  }

  rfd900.Quit();
  */
  /*
  ::src::controls::ground_controls::GroundControls ground_controls;
  ::std::thread rfd900_thread(
      &::src::controls::ground_controls::GroundControls::ReadRFD900,
      &ground_controls);
  ::ros::spin();
  ground_controls.running_ = false;
  rfd900_thread.join();
  */
  ::src::controls::ground_controls::GroundControls ground_controls;
  ::std::thread udp_thread(
      &::src::controls::ground_controls::GroundControls::ReadUDP,
      &ground_controls);
  ::ros::spin();
  ground_controls.running_ = false;
  udp_thread.join();

  // while (true) {
  //   if (!::ros::ok()) {
  //     break;
  //   }

  //   // Run ground controls iteration
  //   ground_controls.RunIteration();

  //   ::ros::spinOnce();
  //   loop.sleep();
  // }

  return 0;
}
