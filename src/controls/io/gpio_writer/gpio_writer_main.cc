#include "gpio_writer.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_gpio_writer");
  ::ros::start();

#ifndef UAS_AT_UCLA_DEPLOYMENT
  if (::ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ::ros::console::levels::Debug)) {
    ::ros::console::notifyLoggerLevelsChanged();
  }
#endif

  ::src::controls::io::gpio_writer::GpioWriter gpio_writer;
  ::ros::spin();
}
