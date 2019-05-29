#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>

#include <linux/limits.h>
#include <ros/console.h>
#include <ros/ros.h>

#ifdef UAS_AT_UCLA_DEPLOYMENT
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

#include "lib/alarm/alarm.h"
#include "lib/phased_loop/phased_loop.h"
#include "src/controls/io/led_strip/led_strip.h"
#include "src/controls/io/ros_to_proto/ros_to_proto.h"
#include "src/controls/ground_controls/timeline/timeline_grammar.pb.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace io {
namespace {
// WiringPi GPIO identifiers
static const int kAlarmGPIOPin = 0;

// Pigpio GPIO identifiers (same as BCM)
static const int kDeploymentEncoderChannelOne = 8;
static const int kDeploymentEncoderChannelTwo = 7;

static const int kDeploymentMotorReverseGPIOPin = 11;
static const int kDeploymentHotwireGPIOPin = 9;
static const int kGimbalGPIOPin = 23;
static const int kDeploymentLatchServoGPIOPin = 24;
static const int kDeploymentMotorGPIOPin = 27;

static const int kPpmMiddleSignal = 1500;

static const int kDeploymentServoClosed = 1600;
static const int kDeploymentServoOpen = 1000;

// Actuator RC channels.
static const int kAlarmOverrideRcChannel = 7;
static const int kDeploymentMotorRcChannel = 8;
static const int kGimbalMotorRcChannel = 9;

static const int kAlarmOverrideRcSignalThreshold = 1800;

static const int kWriterPhasedLoopFrequency = 250;
static const double kRcInTimeGap = 1.0 / 5;

static const int kSensorsPublisherRate = 50;
static const double kSensorsPublisherPeriod = 1.0 / kSensorsPublisherRate;

static const double kAlarmChirpDuration = 0.005;

static const int kRosMessageQueueSize = 1;
static const ::std::string kRosAlarmTriggerTopic = "/uasatucla/actuators/alarm";
static const ::std::string kRosSensorsTopic = "/uasatucla/proto/sensors";
static const ::std::string kRosOutputTopic = "/uasatucla/proto/output";

static const ::std::string kRosRcInTopic = "/mavros/rc/in";
static const ::std::string kRosBatteryStatusTopic = "/mavros/battery";
static const ::std::string kRosStateTopic = "/mavros/state";
static const ::std::string kRosImuTopic = "/mavros/imu/data";

static const ::std::string kRosGlobalPositionTopic =
    "/mavros/setpoint_position/global";

static const ::std::string kRosSetModeService = "/mavros/set_mode";
static const ::std::string kRosArmService = "/mavros/cmd/arming";
static const ::std::string kRosTakeoffService = "/mavros/cmd/takeoff";

} // namespace

class IO {
 public:
  IO();
  void Quit(int sig);

 private:
  void WriterThread();

  void Output(const ::src::controls::Output output);
  void AlarmTriggered(const ::src::controls::AlarmSequence alarm_sequence);
  void RcInReceived(const ::mavros_msgs::RCIn rc_in);
  void BatteryStatusReceived(const ::sensor_msgs::BatteryState battery_state);
  void StateReceived(const ::mavros_msgs::State state);
  void ImuReceived(const ::sensor_msgs::Imu imu);
  void DroneProgramReceived(const ::src::controls::ground_controls::timeline::DroneProgram drone_program);

  ::lib::alarm::Alarm alarm_;
  ::src::controls::io::led_strip::LedStrip led_strip_;

  ros_to_proto::RosToProto ros_to_proto_;
  double next_sensors_write_;

  bool should_override_alarm_;
  double last_rc_in_;

  double deployment_motor_setpoint_;
  double gimbal_setpoint_;
  double deployment_servo_setpoint_;

  bool did_arm_;
  ::std::string px4_mode_;

  ::src::controls::ground_controls::timeline::DroneProgram drone_program_;

  void FlyToLocation();

  ::std::atomic<bool> running_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Publisher sensors_publisher_;
  ::ros::Publisher global_position_publisher_;

  ::ros::Subscriber output_subscriber_;
  ::ros::Subscriber alarm_subscriber_;
  ::ros::Subscriber rc_input_subscriber_;
  ::ros::Subscriber battery_status_subscriber_;
  ::ros::Subscriber state_subscriber_;
  ::ros::Subscriber imu_subscriber_;
  ::ros::Subscriber drone_program_subscriber_;

  ::ros::ServiceClient set_mode_service_;
  ::ros::ServiceClient arm_service_;
  ::ros::ServiceClient takeoff_service_;

  double fly_start_time;
  bool did_trigger_takeoff;
  bool did_takeoff;
  bool did_finish_takeoff;
  bool did_offboard;
  bool did_mission;
  bool did_land;
  double last_msg;

#ifdef UAS_AT_UCLA_DEPLOYMENT
  int pigpio_;
#endif

  ::std::thread writer_thread_;
  ::ros::Rate writer_phased_loop_;
};

} // namespace io
} // namespace controls
} // namespace src
