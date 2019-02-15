#include "gpio_writer.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_gpio_writer");
  ::ros::start();

  ::src::controls::io::gpio_writer::GpioWriter gpio_writer;
  ::ros::spin();
}
