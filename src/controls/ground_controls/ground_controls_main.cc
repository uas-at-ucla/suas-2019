#include "ground_controls.h"
#include "lib/serial_device/serial_device.h"
#include "src/controls/messages.pb.h"
#include <ros/ros.h>
#include <vector>

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_ground_controls");
  ::ros::start();

  ::src::controls::ground_controls::GroundControls ground_controls(argc, argv);
  ::std::thread rfd900_thread(
      &::src::controls::ground_controls::GroundControls::ReadRFD900,
      &ground_controls);
  ::ros::spin();
  ground_controls.running_ = false;
  rfd900_thread.join();

  return 0;
}
