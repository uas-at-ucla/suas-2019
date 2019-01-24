#include "flight_loop.h"

int main() {
  ros::Time::init();

  ::src::controls::loops::FlightLoop flight_loop;
  flight_loop.Run();
}
