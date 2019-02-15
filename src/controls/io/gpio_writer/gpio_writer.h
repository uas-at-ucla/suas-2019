#pragma once

#include <functional>
#include <iostream>
#include <thread>

#include <ros/console.h>
#include <ros/ros.h>

#ifdef UAS_AT_UCLA_DEPLOYMENT
#include <pigpiod_if2.h>
#include <wiringPi.h>
#endif

#include <mavros_msgs/RCIn.h>

#include "lib/alarm/alarm.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace io {
namespace gpio_writer {
namespace {
const int kAlarmGPIOPin = 0;
const int kGimbalGPIOPin = 18;

const int kAlarmOverrideRcChannel = 7;
const int kAlarmOverrideRcSignalThreshold = 1800;

const int kWriterThreadLogIntervalSeconds = 10;

const int kWriterPhasedLoopFrequency = 250;
const ::ros::Duration kAlarmOverrideTimeGap = ::ros::Duration(1.0 / 10);

const int kLedWriterFramesPerSecond = 30;
const ::ros::Duration kLedWriterPeriod = ::ros::Duration(1.0 / kLedWriterFramesPerSecond);

const int kRosMessageQueueSize = 1;
const char *kRosAlarmTriggerTopic = "/uasatucla/actuators/alarm";
const char *kRosRcInTopic = "/mavros/rc/in";
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
  int pigpio_;
#endif
};

} // namespace gpio_writer
} // namespace io
} // namespace controls
} // namespace src
