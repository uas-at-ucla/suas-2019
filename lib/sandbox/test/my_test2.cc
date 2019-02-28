#include "std_msgs/String.h"
#include <ros/console.h>
#include <ros/ros.h>

#include "src/controls/messages.pb.h"

void proto_handler(const ::src::controls::Output output) {
  ::std::cout << output.velocity_x() << ::std::endl;
}

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "my_test2");
  ::ros::start();

  ::ros::NodeHandle n;
  ::ros::Subscriber sub = n.subscribe("output", 1000, proto_handler);

  ::ros::spin();

  return 0;
}
