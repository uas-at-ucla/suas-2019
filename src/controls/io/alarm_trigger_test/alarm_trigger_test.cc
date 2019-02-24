#include <ros/ros.h>

#include <iostream>

#include "src/controls/messages.pb.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "uasatucla_alarm_trigger_test");
  ::ros::start();

  ::ros::NodeHandle ros_node_handle;
  ::ros::Publisher alarm_publisher =
      ros_node_handle.advertise<::src::controls::AlarmSequence>(
          "/uasatucla/actuators/alarm", 1);

  ::src::controls::AlarmSequence alarm_sequence;
  alarm_sequence.add_on_off_cycles(0.005);
  alarm_sequence.add_on_off_cycles(0.005);

  ::ros::Rate phased_loop1(100);
  while (!::ros::master::check()) {
    ::ros::spinOnce();
    phased_loop1.sleep();
  }

  ::ros::Rate phased_loop2(0.1);

  while (::ros::ok()) {
    ::std::cout << "WROTE SOMETHING\n";
    alarm_publisher.publish(alarm_sequence);
    ::ros::spinOnce();

    phased_loop2.sleep();
  }
}
