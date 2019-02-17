#pragma once

#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include <ros/console.h>
#include <ros/ros.h>

#ifdef UAS_AT_UCLA_DEPLOYMENT
#include <pigpiod_if2.h>
#include <wiringPi.h>
#endif

#include <mavros_msgs/RCIn.h>

#include "src/controls/messages.pb.h"
#ifdef UAS_AT_UCLA_DEPLOYMENT
#include "src/controls/io/gpio_writer/led_strip/led_strip.h"
#endif
#include "lib/alarm/alarm.h"

namespace src {
namespace controls {
namespace io {
namespace gpio_writer {
namespace {
static const int kAlarmGPIOPin = 0;
static const int kGimbalGPIOPin = 18;

static const int kAlarmOverrideRcChannel = 7;
static const int kAlarmOverrideRcSignalThreshold = 1800;

static const int kWriterThreadLogIntervalSeconds = 10;

static const int kWriterPhasedLoopFrequency = 250;
static const ::ros::Duration kAlarmOverrideTimeGap = ::ros::Duration(1.0 / 10);

static const int kLedWriterFramesPerSecond = 30;
static const ::ros::Duration kLedWriterPeriod =
    ::ros::Duration(1.0 / kLedWriterFramesPerSecond);

static const double kStartupChirpDuration = 0.005;

static const int kRosMessageQueueSize = 1;
static const ::std::string kRosAlarmTriggerTopic = "/uasatucla/actuators/alarm";
static const ::std::string kRosRcInTopic = "/mavros/rc/in";
} // namespace

class GpioWriter {
 public:
  GpioWriter();

 private:
  void WriterThread();

  void AlarmTriggered(const ::src::controls::AlarmSequence alarm_sequence);
  void RcInReceived(const ::mavros_msgs::RCIn rc_in);

  ::std::thread writer_thread_;
  ::ros::Rate writer_phased_loop_;
  ::ros::Time next_led_write_;

  ::lib::alarm::Alarm alarm_;
  bool should_override_alarm_;
  ::ros::Time last_alarm_override_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber alarm_subscriber_;
  ::ros::Subscriber rc_input_subscriber_;

#ifdef UAS_AT_UCLA_DEPLOYMENT
  led_strip::LedStrip led_strip_;
  int pigpio_;
#endif
};

} // namespace gpio_writer
} // namespace io
} // namespace controls
} // namespace src
