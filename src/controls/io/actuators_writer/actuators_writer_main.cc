#include "actuators_writer.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_actuators_writer");
  ::ros::start();
 
  if(::ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ::ros::console::levels::Debug) ) {
    ::ros::console::notifyLoggerLevelsChanged();
  }

  ::src::controls::io::actuators_writer::ActuatorsWriter actuators_writer;
  ::ros::spin();
}
