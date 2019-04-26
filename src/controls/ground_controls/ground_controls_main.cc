// #include "ground_controls.h"
#include <ros/ros.h>
#include "lib/serial_device/serial_device.h"
#include <vector>
#include "src/controls/messages.pb.h"

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  ::ros::init(argc, argv, "uasatucla_ground_controls");
  ::ros::start();

  // ::src::controls::ground_controls::GroundControls ground_controls;
  // ground_controls.Run();

  // Serial device test
  ::lib::serial_device::SerialDevice<::src::controls::UasMessage> rfd900("/dev/ttyUSB0", B57600, 0);
  
  ::ros::Rate loop(50);

  ::src::controls::UasMessage current_message;

  while(true) {
    if(!::ros::ok()) {
      break;
    }

    if(rfd900.GetProtoMessage(current_message) && current_message.has_sensors()) {
      ::std::cout << "Battery Voltage: " << current_message.sensors().battery_voltage() << " Latitude: " << current_message.sensors().latitude() << " Longitude: " << current_message.sensors().longitude() << " Altitude: " << current_message.sensors().altitude() << ::std::endl;
    }

    ::ros::spinOnce();
    loop.sleep();
  }

  rfd900.Quit();

  return 0;
}
