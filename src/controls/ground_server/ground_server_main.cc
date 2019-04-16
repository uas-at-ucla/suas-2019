#include "ground_server.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_ground_server");
  ::ros::start();

  ::src::controls::ground_server::GroundServer drone_server;
  ::ros::spin();
}
