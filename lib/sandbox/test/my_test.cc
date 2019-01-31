#include "std_msgs/String.h"
#include <ros/console.h>
#include <ros/ros.h>

#include "src/controls/messages.pb.h"

int main(int argc, char **argv) {
  ::ros::init(argc, argv, "my_test");
  ::ros::start();
  ::ros::Rate loop(10);

  ::ros::NodeHandle n;
  ::ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ::ros::Publisher proto_pub =
      n.advertise<::src::controls::Output>("output", 1000);

  for (int i = 0; i < 10000 && ::ros::ok(); i++) {
    ::std::cout << "Here " << i << ::std::endl;
    ROS_INFO_STREAM("Hello "
                    << "World " << i);

    ::src::controls::Output output;
    output.set_state(0);
    output.set_flight_time(0);
    output.set_current_command_index(0);

    output.set_velocity_x(i);
    output.set_velocity_y(0);
    output.set_velocity_z(0);
    output.set_yaw_setpoint(0);

    output.set_gimbal_angle(0);
    output.set_bomb_drop(false);
    output.set_alarm(false);
    output.set_dslr(false);

    output.set_trigger_takeoff(0);
    output.set_trigger_hold(0);
    output.set_trigger_offboard(0);
    output.set_trigger_rtl(0);
    output.set_trigger_land(0);
    output.set_trigger_arm(0);
    output.set_trigger_disarm(0);

    proto_pub.publish(output);

    std_msgs::String msg;
    msg.data = "test";
    chatter_pub.publish(msg);

    loop.sleep();
  }

  return 0;
}
