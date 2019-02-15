#pragma once

#include <functional>
#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <ros/console.h>

#ifdef UAS_AT_UCLA_DEPLOYMENT
#include <pigpiod_if2.h>
#include <wiringPi.h>
#endif

#include "src/controls/messages.pb.h"
#include "lib/alarm/alarm.h"

namespace src {
namespace controls {
namespace io {
namespace actuators_writer {
namespace {
const int kAlarmGPIOPin = 0;
const int kGimbalGPIOPin = 18;

const int kWriterPhasedLoopFrequency = 250;
} // namespace

class ActuatorsWriter {
 public:
  ActuatorsWriter();

 private:
  void WriterThread();
  void AlarmTriggered(const ::src::controls::AlarmSequence alarm_sequence);

  ::std::thread writer_thread_;
  ::ros::Rate writer_phased_loop_;

  ::lib::alarm::Alarm alarm_;

  ::ros::NodeHandle ros_node_handle_;
  ::ros::Subscriber alarm_subscriber_;

#ifdef UAS_AT_UCLA_DEPLOYMENT
  int pigpio_;
#endif
};

} // namespace actuators_writer
} // namespace io
} // namespace controls
} // namespace src
