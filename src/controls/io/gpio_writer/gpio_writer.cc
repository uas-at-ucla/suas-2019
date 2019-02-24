#include "gpio_writer.h"

namespace src {
namespace controls {
namespace io {
namespace gpio_writer {

GpioWriter::GpioWriter() :
    writer_thread_(&GpioWriter::WriterThread, this),
    writer_phased_loop_(kWriterPhasedLoopFrequency),
    next_led_write_(::lib::phased_loop::GetCurrentTime()),
    alarm_(kWriterPhasedLoopFrequency),
    should_override_alarm_(false),
    last_alarm_override_(::lib::phased_loop::GetCurrentTime()),
    did_arm_(false),
    alarm_subscriber_(
        ros_node_handle_.subscribe(kRosAlarmTriggerTopic, kRosMessageQueueSize,
                                   &GpioWriter::AlarmTriggered, this)),
    rc_input_subscriber_(ros_node_handle_.subscribe(
        kRosRcInTopic, kRosMessageQueueSize, &GpioWriter::RcInReceived, this)),
    battery_status_subscriber_(
        ros_node_handle_.subscribe(kRosBatteryStatusTopic, kRosMessageQueueSize,
                                   &GpioWriter::BatteryStatusReceived, this)),
    state_subscriber_(
        ros_node_handle_.subscribe(kRosStateTopic, kRosMessageQueueSize,
                                   &GpioWriter::StateReceived, this)),
    imu_subscriber_(ros_node_handle_.subscribe(
        kRosImuTopic, kRosMessageQueueSize, &GpioWriter::ImuReceived, this)) {

#ifdef UAS_AT_UCLA_DEPLOYMENT
  // Alarm IO setup.
  wiringPiSetup();
  pinMode(kAlarmGPIOPin, OUTPUT);
#endif

  // alarm_.AddAlert({kAlarmChirpDuration, 0});
}

void GpioWriter::WriterThread() {
  while (::ros::ok()) {
    // Write out the alarm signal.
    bool should_override_alarm = (should_override_alarm_ &&
                                  last_alarm_override_ + kAlarmOverrideTimeGap >
                                      ::lib::phased_loop::GetCurrentTime());
    bool should_alarm = alarm_.ShouldAlarm() || should_override_alarm;

    led_strip_.set_alarm(should_override_alarm);

#ifdef UAS_AT_UCLA_DEPLOYMENT
    digitalWrite(kAlarmGPIOPin, should_alarm ? HIGH : LOW);
#endif

    if (::lib::phased_loop::GetCurrentTime() > next_led_write_) {
      led_strip_.Render();

      next_led_write_ = ::lib::phased_loop::GetCurrentTime() + kLedWriterPeriod;
    }

    ROS_DEBUG_STREAM("Writer thread iteration: "
                     << ::std::endl
                     << "alarm[" << should_alarm << "]" << ::std::endl
                     << "led_strip[" << led_strip_.GetStrip() << ::std::endl
                     << "]");

    // Wait until next iteration of loop.
    writer_phased_loop_.sleep();
  }
}

void GpioWriter::AlarmTriggered(
    const ::src::controls::AlarmSequence alarm_sequence) {
  ROS_DEBUG_STREAM("Alarm triggered via ROS.");

  // Override any current alarms.
  alarm_.ClearAlerts();

  // Iterate through all on-off pairs and add them to the alarm sequence.
  for (int i = 0; i < alarm_sequence.on_off_cycles_size() -
                          alarm_sequence.on_off_cycles_size() % 2;
       i += 2) {
    alarm_.AddAlert(
        {alarm_sequence.on_off_cycles(i), alarm_sequence.on_off_cycles(i + 1)});
  }
}

void GpioWriter::RcInReceived(const ::mavros_msgs::RCIn rc_in) {
  bool new_should_override_alarm = should_override_alarm_;

  // Trigger the alarm if the RC controller override switch was flipped.
  if (rc_in.channels[kAlarmOverrideRcChannel - 1] >
      kAlarmOverrideRcSignalThreshold) {
    new_should_override_alarm = true;
    last_alarm_override_ = ::lib::phased_loop::GetCurrentTime();
  } else {
    new_should_override_alarm = false;
  }

  // Record a log message on every edge.
  if (new_should_override_alarm != should_override_alarm_) {
    ROS_INFO_STREAM("Alarm RC override changed: " << should_override_alarm_
                                                  << " -> "
                                                  << new_should_override_alarm);
  }

  should_override_alarm_ = new_should_override_alarm;
}

void GpioWriter::BatteryStatusReceived(
    const ::sensor_msgs::BatteryState battery_state) {
  led_strip_.set_battery_percentage(battery_state.percentage);
}

void GpioWriter::StateReceived(const ::mavros_msgs::State state) {
  if (state.armed != did_arm_) {
    ROS_INFO_STREAM("Arming state changed: "
                    << (did_arm_ ? "ARMED" : "DISARMED") << " -> "
                    << (state.armed ? "ARMED" : "DISARMED"));

    if (state.armed) {
      // Send out the armed chirp.
      // alarm_.AddAlert({kAlarmChirpDuration, 0});
    }

    did_arm_ = state.armed;
  }

  led_strip_.set_armed(state.armed);
}

void GpioWriter::ImuReceived(const ::sensor_msgs::Imu imu) {
  (void)imu;
  led_strip_.set_last_imu(::lib::phased_loop::GetCurrentTime());
}

} // namespace gpio_writer
} // namespace io
} // namespace controls
} // namespace src