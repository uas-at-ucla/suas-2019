// #include "ground_controls.h"
#include <ros/ros.h>
#include "lib/serial_device/serial_device.h"
#include <vector>
#include "src/controls/messages.pb.h"

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  // ::ros::init(argc, argv, "uasatucla_ground_controls");
  // ::ros::start();

  // ::src::controls::ground_controls::GroundControls ground_controls;
  // ground_controls.Run();

  // Serial device test
  ::lib::serial_device::SerialDevice<::src::controls::Sensors> rfd900("/dev/ttyUSB0", 57600, 0);
  
  for(int i = 0;i < 1000;i++) {
    rfd900.WritePort("hello world");
    rfd900.WritePort("hello");
    rfd900.WritePort("hello ");
  }

  rfd900.Quit();

  return 0;
}
