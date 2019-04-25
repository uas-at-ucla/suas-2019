#include "ground_controls.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_ground_controls");
  ::ros::start();

  ::src::controls::ground_communicator::GroundControls ground_controls;
  ground_controls.Run();

  // Serial device test
  ::src::controls::ground_communicator::SerialDevice("/dev/ttyUSB0", 57600, 0)
}
