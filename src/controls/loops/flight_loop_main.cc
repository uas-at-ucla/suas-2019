#include "flight_loop.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_flight_loop");
  ::ros::start();

  ::src::controls::loops::FlightLoop flight_loop;
  ::ros::spin();
}
