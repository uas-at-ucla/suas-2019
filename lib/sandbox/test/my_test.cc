#include "std_msgs/String.h"
#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "my_test");
  ::ros::start();
  ::ros::Rate loop(100);

  ::ros::NodeHandle n;
  ::ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  for (int i = 0; i < 10000 && ::ros::ok(); i++) {
    ::std::cout << "Here " << i << ::std::endl;
    loop.sleep();
    ROS_INFO_STREAM("Hello "
                    << "World " << i);

    std_msgs::String msg;
    msg.data = "test";
    chatter_pub.publish(msg);
  }

  return 0;
}
