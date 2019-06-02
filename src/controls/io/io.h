#pragma once

#include <atomic>
#include <functional>
#include <math.h>
#include <string>
#include <thread>

#include <GeographicLib/Geoid.hpp>
#include <linux/limits.h>
#include <ros/console.h>
#include <ros/ros.h>

#ifdef RASPI_DEPLOYMENT
#include <pigpiod_if2.h>
#include <softPwm.h>
#include <wiringPi.h>
#endif

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "lib/alarm/alarm.h"
#include "lib/deployment/deployment.h"
#include "lib/phased_loop/phased_loop.h"
#include "src/controls/ground_controls/timeline/timeline_grammar.pb.h"
#include "src/controls/io/led_strip/led_strip.h"
#include "src/controls/io/ros_to_proto/ros_to_proto.h"
#include "src/controls/messages.pb.h"

#include "src/controls/constants.h"

// #define LOG_LED_STRIP 0

namespace src {
namespace controls {
namespace io {

class IO {
 public:
  IO();
  void Quit(int signal);

 private:
  void WriterThread();

  void GimbalSetpoint(const ::std_msgs::Float32 gimbal_setpoint);
  void
  DeploymentMotorSetpoint(const ::std_msgs::Float32 deployment_motor_setpoint);
  void LatchSetpoint(const ::std_msgs::Bool latch_setpoint);
  void HotwireSetpoint(const ::std_msgs::Bool hotwire_setpoint);
  void AlarmTriggered(const ::src::controls::AlarmSequence alarm_sequence);

  void RcInReceived(const ::mavros_msgs::RCIn rc_in);
  void BatteryStatusReceived(const ::sensor_msgs::BatteryState battery_state);
  void StateReceived(const ::mavros_msgs::State state);
  void ImuReceived(const ::sensor_msgs::Imu imu);
  void DroneProgramReceived(
      const ::src::controls::ground_controls::timeline::DroneProgram
          drone_program);

  // Actuator setup and write handlers.
  void InitializeActuators();
  void WriteAlarm(bool alarm);
  void WriteGimbal(double pitch);
  void WriteDeployment(::lib::deployment::Output &output);

  void PixhawkSendModePosedge(::std::string mode, bool signal);
  void PixhawkSetGlobalPositionGoal(double latitude, double longitude,
                                    double altitude, double yaw);

  ::lib::alarm::Alarm alarm_;
  ::lib::deployment::Deployment deployment_;
  ::src::controls::io::led_strip::LedStrip led_strip_;

  ros_to_proto::RosToProto ros_to_proto_;
  double next_sensors_write_;

  bool should_override_alarm_;
  double last_rc_in_;

  ::std::atomic<double> gimbal_setpoint_;
  ::std::atomic<double> deployment_motor_setpoint_;
  ::std::atomic<bool> latch_setpoint_;
  ::std::atomic<bool> hotwire_setpoint_;

  bool did_arm_;
  ::std::string px4_mode_;

  ::src::controls::ground_controls::timeline::DroneProgram drone_program_;

  void FlyToLocation();

  ::std::atomic<bool> running_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Publisher sensors_publisher_;
  ::ros::Publisher global_position_publisher_;

  ::ros::Publisher gimbal_publisher_;
  ::ros::Publisher deployment_motor_publisher_;
  ::ros::Publisher latch_publisher_;
  ::ros::Publisher hotwire_publisher_;

  ::ros::Subscriber gimbal_subscriber_;
  ::ros::Subscriber deployment_motor_subscriber_;
  ::ros::Subscriber latch_subscriber_;
  ::ros::Subscriber hotwire_subscriber_;
  ::ros::Subscriber alarm_subscriber_;

  ::ros::Subscriber rc_input_subscriber_;
  ::ros::Subscriber battery_status_subscriber_;
  ::ros::Subscriber state_subscriber_;
  ::ros::Subscriber imu_subscriber_;
  ::ros::Subscriber drone_program_subscriber_;

  ::ros::ServiceClient set_mode_service_;
  ::ros::ServiceClient arm_service_;
  ::ros::ServiceClient takeoff_service_;

  ::std::map<::std::string, bool> last_mode_signals_;
  double last_position_setpoint_;

  bool last_arm_state_;

#ifdef RASPI_DEPLOYMENT
  int pigpio_;
#endif

  ::std::thread writer_thread_;
  ::ros::Rate writer_phased_loop_;
};

} // namespace io
} // namespace controls
} // namespace src
