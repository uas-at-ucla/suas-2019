#include "io.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_io");
  ::ros::start();

#ifndef UAS_AT_UCLA_DEPLOYMENT
  // Log more verbose output if running in SITL.
  if (::ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ::ros::console::levels::Debug)) {
    ::ros::console::notifyLoggerLevelsChanged();
  }
#endif

  ::src::controls::io::IO io;
  ::ros::spin();
}
