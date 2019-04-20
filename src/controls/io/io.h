#pragma once

#include <functional>
#include <string>
#include <thread>

#include <ros/console.h>
#include <ros/ros.h>

#ifdef UAS_AT_UCLA_DEPLOYMENT
#include <pigpiod_if2.h>
#include <wiringPi.h>
#include <softPwm.h>
#endif

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

#include "lib/alarm/alarm.h"
#include "lib/phased_loop/phased_loop.h"
#include "src/controls/io/led_strip/led_strip.h"
#include "src/controls/io/ros_to_proto/ros_to_proto.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace io {
namespace {
static const int kAlarmGPIOPin = 0;
static const int kDeploymentGPIOPin = 2;
static const int kGimbalGPIOPin = 23;

static const int kAlarmOverrideRcChannel = 7;
static const int kAlarmOverrideRcSignalThreshold = 1800;

static const int kWriterPhasedLoopFrequency = 250;
static const double kAlarmOverrideTimeGap = 1.0 / 10;

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
} // namespace

class IO {
 public:
  IO();

 private:
  void WriterThread();

  void Output(const ::src::controls::Output output);
  void AlarmTriggered(const ::src::controls::AlarmSequence alarm_sequence);
  void RcInReceived(const ::mavros_msgs::RCIn rc_in);
  void BatteryStatusReceived(const ::sensor_msgs::BatteryState battery_state);
  void StateReceived(const ::mavros_msgs::State state);
  void ImuReceived(const ::sensor_msgs::Imu imu);

  ::lib::alarm::Alarm alarm_;

  led_strip::LedStrip led_strip_;

  ros_to_proto::RosToProto ros_to_proto_;
  double next_sensors_write_;

  bool should_override_alarm_;
  double last_alarm_override_;

  bool did_arm_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Publisher sensors_publisher_;
  ::ros::Subscriber output_subscriber_;
  ::ros::Subscriber alarm_subscriber_;
  ::ros::Subscriber rc_input_subscriber_;
  ::ros::Subscriber battery_status_subscriber_;
  ::ros::Subscriber state_subscriber_;
  ::ros::Subscriber imu_subscriber_;

#ifdef UAS_AT_UCLA_DEPLOYMENT
  int pigpio_;
#endif

  ::std::thread writer_thread_;
  ::ros::Rate writer_phased_loop_;
};

} // namespace io
} // namespace controls
} // namespace src
