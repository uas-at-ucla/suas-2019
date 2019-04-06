#include "ground_communicator.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_ground_communicator");
  ::ros::start();

  ::src::controls::ground_communicator::GroundCommunicator ground_communicator;
  ground_communicator.Run();
}
