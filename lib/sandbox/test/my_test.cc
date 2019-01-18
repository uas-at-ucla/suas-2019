#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "my_test");
  ::ros::start();
  ::ros::Rate loop(100);

  for (int i = 0; i < 1000; i++) {
    loop.sleep();
    ROS_INFO_STREAM("Hello "
                    << "World " << i);
  }

  return 0;
}
