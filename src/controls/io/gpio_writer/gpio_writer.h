#pragma once

#include <functional>
#include <string>
#include <thread>

#include <ros/console.h>
#include <ros/ros.h>

#ifdef UAS_AT_UCLA_DEPLOYMENT
#include <pigpiod_if2.h>
#include <wiringPi.h>
#endif

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>

#include "lib/alarm/alarm.h"
#include "lib/phased_loop/phased_loop.h"
#include "src/controls/io/gpio_writer/led_strip/led_strip.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace io {
namespace gpio_writer {
namespace {
static const int kAlarmGPIOPin = 0;
static const int kGimbalGPIOPin = 18;

static const int kAlarmOverrideRcChannel = 7;
static const int kAlarmOverrideRcSignalThreshold = 1800;

static const int kWriterPhasedLoopFrequency = 250;
static const double kAlarmOverrideTimeGap = 1.0 / 10;

static const int kLedWriterFramesPerSecond = 30;
static const double kLedWriterPeriod = 1.0 / kLedWriterFramesPerSecond;

static const double kAlarmChirpDuration = 0.010;

static const int kRosMessageQueueSize = 1;
static const ::std::string kRosAlarmTriggerTopic = "/uasatucla/actuators/alarm";
static const ::std::string kRosRcInTopic = "/mavros/rc/in";
static const ::std::string kRosBatteryStatusTopic = "/mavros/battery";
static const ::std::string kRosStateTopic = "/mavros/state";
} // namespace

class GpioWriter {
 public:
  GpioWriter();

 private:
  void WriterThread();

  void AlarmTriggered(const ::src::controls::AlarmSequence alarm_sequence);
  void RcInReceived(const ::mavros_msgs::RCIn rc_in);
  void BatteryStatusReceived(const ::sensor_msgs::BatteryState battery_state);
  void StateReceived(const ::mavros_msgs::State state);

  led_strip::LedStrip led_strip_;

  ::std::thread writer_thread_;
  ::ros::Rate writer_phased_loop_;
  double next_led_write_;

  ::lib::alarm::Alarm alarm_;
  bool should_override_alarm_;
  double last_alarm_override_;

  bool did_arm_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber alarm_subscriber_;
  ::ros::Subscriber rc_input_subscriber_;
  ::ros::Subscriber battery_status_subscriber_;
  ::ros::Subscriber state_subscriber_;

#ifdef UAS_AT_UCLA_DEPLOYMENT
  int pigpio_;
#endif
};

} // namespace gpio_writer
} // namespace io
} // namespace controls
} // namespace src
