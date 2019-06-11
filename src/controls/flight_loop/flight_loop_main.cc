#include "flight_loop.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_flight_loop");
  ::ros::start();

  // #ifndef RASPI_DEPLOYMENT
  // Log more verbose output if running in SITL.
  if (::ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ::ros::console::levels::Debug)) {
    ::ros::console::notifyLoggerLevelsChanged();
  }
  // #endif

  ::src::controls::flight_loop::FlightLoop flight_loop;
  ::ros::spin();
}
