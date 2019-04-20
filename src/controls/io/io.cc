#include "io.h"

namespace src {
namespace controls {
namespace io {

IO::IO() :
    alarm_(kWriterPhasedLoopFrequency),
    next_sensors_write_(::lib::phased_loop::GetCurrentTime()),
    should_override_alarm_(false),
    last_alarm_override_(::lib::phased_loop::GetCurrentTime()),
    did_arm_(false),
    sensors_publisher_(ros_node_handle_.advertise<::src::controls::Sensors>(
        kRosSensorsTopic, kRosMessageQueueSize)),
    output_subscriber_(ros_node_handle_.subscribe(
        kRosOutputTopic, kRosMessageQueueSize, &IO::Output, this)),
    alarm_subscriber_(ros_node_handle_.subscribe(kRosAlarmTriggerTopic,
                                                 kRosMessageQueueSize,
                                                 &IO::AlarmTriggered, this)),
    rc_input_subscriber_(ros_node_handle_.subscribe(
        kRosRcInTopic, kRosMessageQueueSize, &IO::RcInReceived, this)),
    battery_status_subscriber_(
        ros_node_handle_.subscribe(kRosBatteryStatusTopic, kRosMessageQueueSize,
                                   &IO::BatteryStatusReceived, this)),
    state_subscriber_(ros_node_handle_.subscribe(
        kRosStateTopic, kRosMessageQueueSize, &IO::StateReceived, this)),
    imu_subscriber_(ros_node_handle_.subscribe(
        kRosImuTopic, kRosMessageQueueSize, &IO::ImuReceived, this)),
    writer_thread_(&IO::WriterThread, this),
    writer_phased_loop_(kWriterPhasedLoopFrequency) {

#ifdef UAS_AT_UCLA_DEPLOYMENT
  // Alarm IO setup.
  wiringPiSetup();
  pinMode(kAlarmGPIOPin, OUTPUT);
  ::std::cout << "CREATE: " << softPwmCreate(2, 0, 100) << ::std::endl;
#endif

  // Chirp when the io program starts.
  alarm_.AddAlert({kAlarmChirpDuration, 0});
}

void IO::WriterThread() {
  while (::ros::ok()) {
    // Write out the alarm signal.
    bool should_override_alarm = (should_override_alarm_ &&
                                  last_alarm_override_ + kAlarmOverrideTimeGap >
                                      ::lib::phased_loop::GetCurrentTime());
    bool should_alarm = alarm_.ShouldAlarm() || should_override_alarm;

    led_strip_.set_alarm(should_override_alarm);

#ifdef UAS_AT_UCLA_DEPLOYMENT
    digitalWrite(kAlarmGPIOPin, should_alarm ? HIGH : LOW);
    // static int i = 0;
    // static bool up = true;
    // if(up) {
    //   i++;
    //   if(i > 999) {
    //     up = false;
    //   }
    // } else {
    //   i--;
    //   if(i < 1) {
    //     up = true;
    //   }
    // }
    // softPwmWrite(2, i / 10);
    // if(i % 10 == 0) {
    //   ::std::cout << "sending " << i << ::std::endl;
    // }
#endif

    // Write output to LED strip.
    led_strip_.Render();

    // Write out sensors protobuf at a slower rate than the write loop.
    if (::lib::phased_loop::GetCurrentTime() > next_sensors_write_ &&
        ros_to_proto_.SensorsValid()) {
      sensors_publisher_.publish(ros_to_proto_.GetSensors());

      next_sensors_write_ =
          ::lib::phased_loop::GetCurrentTime() + kSensorsPublisherPeriod;
    }

    // Log the current GPIO outputs.
    ROS_DEBUG_STREAM("Writer thread iteration: "
                     << ::std::endl
                     << "alarm[" << should_alarm << "]" << ::std::endl
                     << "led_strip[" << led_strip_.GetStrip() << ::std::endl
                     << "]");

    // Wait until next iteration of loop.
    writer_phased_loop_.sleep();
  }
}

void IO::Output(const ::src::controls::Output output) {
  ROS_DEBUG_STREAM(
      "Got output protobuf from flight_loop. vx: " << output.velocity_x());
}

void IO::AlarmTriggered(const ::src::controls::AlarmSequence alarm_sequence) {
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

void IO::RcInReceived(const ::mavros_msgs::RCIn rc_in) {
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

void IO::BatteryStatusReceived(
    const ::sensor_msgs::BatteryState battery_state) {
  led_strip_.set_battery_percentage(battery_state.percentage);
}

void IO::StateReceived(const ::mavros_msgs::State state) {
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

void IO::ImuReceived(const ::sensor_msgs::Imu imu) {
  (void)imu;
  led_strip_.set_last_imu(::lib::phased_loop::GetCurrentTime());
}

} // namespace io
} // namespace controls
} // namespace src
