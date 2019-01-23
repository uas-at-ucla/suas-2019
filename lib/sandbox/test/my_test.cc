#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "my_test");
  ::ros::start();
  ::ros::Rate loop(10);

  for (int i = 0; i < 100; i++) {
    ::std::cout << "Here " << i << ::std::endl;
    loop.sleep();
    ROS_INFO_STREAM("Hello "
                    << "World " << i);
  }

  return 0;
}
