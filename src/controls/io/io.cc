#include "io.h"

namespace src {
namespace controls {
namespace io {

void deploymentChannelOneTrigger(int gpio, int level, uint32_t tick) {
  (void)gpio;
  (void)level;
  (void)tick;
}

IO::IO() :
    alarm_(kWriterPhasedLoopFrequency),
    next_sensors_write_(::lib::phased_loop::GetCurrentTime()),
    should_override_alarm_(false),
    last_rc_in_(::lib::phased_loop::GetCurrentTime()),
    deployment_motor_setpoint_(0.0),
    gimbal_setpoint_(0.0),
    deployment_servo_setpoint_(kDeploymentServoClosed),
    running_(true),
    sensors_publisher_(ros_node_handle_.advertise<::src::controls::Sensors>(
        kRosSensorsTopic, kRosMessageQueueSize)),
    global_position_publisher_(
        ros_node_handle_.advertise<::mavros_msgs::GlobalPositionTarget>(
            kRosGlobalPositionTopic, 10)),
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
    set_mode_service_(ros_node_handle_.serviceClient<::mavros_msgs::SetMode>(
        kRosSetModeService)),
    arm_service_(ros_node_handle_.serviceClient<::mavros_msgs::CommandBool>(
        kRosArmService)),
    takeoff_service_(ros_node_handle_.serviceClient<::mavros_msgs::CommandTOL>(
        kRosTakeoffService)),
    last_arm_(false),
    last_takeoff_(false),
    last_land_(false),
    last_global_position_setpoint_(-std::numeric_limits<double>::infinity()),
    last_arm_state_(false),
    writer_thread_(&IO::WriterThread, this),
    writer_phased_loop_(kWriterPhasedLoopFrequency) {

#ifdef UAS_AT_UCLA_DEPLOYMENT
  // Alarm IO setup.
  wiringPiSetup();

  pigpio_ = pigpio_start(0, 0);
  // Inputs.
  set_mode(pigpio_, kDeploymentEncoderChannelOne, PI_INPUT);
  set_mode(pigpio_, kDeploymentEncoderChannelTwo, PI_INPUT);

  // Outputs.
  set_mode(pigpio_, kGimbalGPIOPin, PI_OUTPUT);
  set_mode(pigpio_, kDeploymentLatchServoGPIOPin, PI_OUTPUT);
  set_mode(pigpio_, kDeploymentMotorGPIOPin, PI_OUTPUT);
  set_mode(pigpio_, kDeploymentMotorReverseGPIOPin, PI_OUTPUT);
  set_mode(pigpio_, kDeploymentHotwireGPIOPin, PI_OUTPUT);
  set_PWM_frequency(pigpio_, kDeploymentMotorGPIOPin, 400);
  set_PWM_range(pigpio_, kDeploymentMotorGPIOPin, 100);

  pinMode(kAlarmGPIOPin, OUTPUT);

  // Set initial values
  set_servo_pulsewidth(pigpio_, kGimbalGPIOPin, kPpmMiddleSignal);
  set_servo_pulsewidth(pigpio_, kDeploymentLatchServoGPIOPin,
                       kDeploymentServoClosed);
  set_PWM_dutycycle(pigpio_, kDeploymentMotorGPIOPin, 0);
  gpio_write(pigpio_, kDeploymentMotorReverseGPIOPin, 0);
  gpio_write(pigpio_, kDeploymentHotwireGPIOPin, 0);

  // Add callback functions for encoder pins.
  gpioSetAlertFunc(kDeploymentEncoderChannelOne, deploymentChannelOneTrigger);
#endif

  // Chirp when the io program starts.
  alarm_.AddAlert({kAlarmChirpDuration, 0});
}

void IO::Quit(int sig) {
  (void)sig;

  running_ = false;
  writer_thread_.join();

#ifdef UAS_AT_UCLA_DEPLOYMENT
  set_servo_pulsewidth(pigpio_, kGimbalGPIOPin, kPpmMiddleSignal);
  set_servo_pulsewidth(pigpio_, kDeploymentLatchServoGPIOPin,
                       kDeploymentServoClosed);
  set_PWM_dutycycle(pigpio_, kDeploymentMotorGPIOPin, 0);
  gpio_write(pigpio_, kDeploymentMotorReverseGPIOPin, 0);
  gpio_write(pigpio_, kDeploymentHotwireGPIOPin, 0);

  digitalWrite(kAlarmGPIOPin, false);
#endif
  led_strip_.set_blank(true);
  led_strip_.Render(true);

  // Sleep a bit so that the outputs are actually written.
  usleep(0.1 * 1e6);

#ifdef UAS_AT_UCLA_DEPLOYMENT
  pigpio_stop(pigpio_);
#endif
}

void IO::WriterThread() {
  while (running_ && ::ros::ok()) {
    // Write out the alarm signal.
    // This was hacked to make the alarm switch trigger deployment!
    /*
    bool should_override_alarm = (should_override_alarm_ &&
                                  last_rc_in_ + kRcInTimeGap >
                                      ::lib::phased_loop::GetCurrentTime());
    bool should_alarm = alarm_.ShouldAlarm() || should_override_alarm;
    */
    bool should_alarm = alarm_.ShouldAlarm();
    (void)should_alarm;

    bool hotwire_setpoint = false;
    static bool last_should_alarm = false;

    // If running in a simulator, trigger the arm/takeoff/land sequence after
    // a certain amount of time.
#ifndef UAS_AT_UCLA_DEPLOYMENT
    static double start = ::lib::phased_loop::GetCurrentTime();
    if (::lib::phased_loop::GetCurrentTime() - start > 5) {
      should_override_alarm_ = true;
    }
#endif

    if (should_override_alarm_) {
      deployment_servo_setpoint_ = kDeploymentServoOpen;
      hotwire_setpoint = true;

      (void)last_should_alarm;
      // if (last_should_alarm != should_override_alarm_) {
      //   fly_start_time = ::lib::phased_loop::GetCurrentTime();
      //   did_arm = false;
      //   did_takeoff = false;
      //   did_land = false;
      //   did_offboard = false;
      //   last_msg = ::lib::phased_loop::GetCurrentTime();
      // }

      // FlyToLocation();
    } else {
      // if (last_should_alarm != should_override_alarm_) {
      //   if (did_offboard) {
      //     ::mavros_msgs::SetMode srv_setMode;
      //     srv_setMode.request.base_mode = 0;
      //     srv_setMode.request.custom_mode = "POSCTL";

      //     if (!set_mode_service_.call(srv_setMode)) {
      //       ROS_ERROR("Failed SetMode");
      //     }
      //   }
      // }

      deployment_servo_setpoint_ = kDeploymentServoClosed;
    }

    last_should_alarm = should_override_alarm_;

    (void)gimbal_setpoint_;
#ifdef UAS_AT_UCLA_DEPLOYMENT
    // Write out alarm.
    digitalWrite(kAlarmGPIOPin, should_alarm ? HIGH : LOW);

    // Write out gimbal.
    set_servo_pulsewidth(pigpio_, kGimbalGPIOPin,
                         1500 + gimbal_setpoint_ * 500);

    // Write out deployment.
    if (deployment_motor_setpoint_ >= 0) {
      set_PWM_dutycycle(pigpio_, kDeploymentMotorGPIOPin,
                        deployment_motor_setpoint_ * 100);
      gpio_write(pigpio_, kDeploymentMotorReverseGPIOPin, 0);
    } else {
      set_PWM_dutycycle(pigpio_, kDeploymentMotorGPIOPin,
                        deployment_motor_setpoint_ * -100);
      gpio_write(pigpio_, kDeploymentMotorReverseGPIOPin, 1);
    }

    gpio_write(pigpio_, kDeploymentHotwireGPIOPin, hotwire_setpoint ? 1 : 0);

    set_servo_pulsewidth(pigpio_, kDeploymentLatchServoGPIOPin,
                         deployment_servo_setpoint_);
#endif

    // Write output to LED strip.
    led_strip_.Render(false);

    // Write out sensors protobuf at a slower rate than the write loop.
    if (::lib::phased_loop::GetCurrentTime() > next_sensors_write_ &&
        ros_to_proto_.SensorsValid()) {
      sensors_publisher_.publish(ros_to_proto_.GetSensors());

      next_sensors_write_ =
          ::lib::phased_loop::GetCurrentTime() + kSensorsPublisherPeriod;
    }

    // Log the current GPIO outputs.
    // ROS_DEBUG_STREAM("Writer thread iteration: "
    //                  << ::std::endl
    //                  << "alarm[" << should_alarm << "]" << ::std::endl
    //                  << "led_strip[" << led_strip_.GetStrip() << ::std::endl
    //                  << "]");

    // Wait until next iteration of loop.
    writer_phased_loop_.sleep();
  }
}

void IO::Output(const ::src::controls::Output output) {
  ROS_DEBUG_STREAM(
      "Got output protobuf from flight_loop. vx: " << output.velocity_x());

  // Only listen to output if safety pilot override is not active.
  bool run_uas_flight_loop = true;
  if (!run_uas_flight_loop) {
    return;
  }
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
  last_rc_in_ = ::lib::phased_loop::GetCurrentTime();

  // Trigger the alarm if the RC controller override switch was flipped.
  if (rc_in.channels[kAlarmOverrideRcChannel - 1] >
      kAlarmOverrideRcSignalThreshold) {
    new_should_override_alarm = true;
  } else {
    new_should_override_alarm = false;
  }

  int deployment_rc_in = rc_in.channels[kDeploymentMotorRcChannel - 1];
  if (deployment_rc_in > 900) {
    deployment_motor_setpoint_ =
        ::std::max(::std::min((deployment_rc_in - 1500) / 500.0, 1.0), -1.0);
    if (::std::abs(deployment_motor_setpoint_) < 0.1) {
      deployment_motor_setpoint_ = 0;
    }
  } else {
    deployment_motor_setpoint_ = 0;
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
  if (state.armed != last_arm_state_) {
    ROS_INFO_STREAM("Arming state changed: "
                    << (last_arm_state_ ? "ARMED" : "DISARMED") << " -> "
                    << (state.armed ? "ARMED" : "DISARMED"));

    last_arm_state_ = state.armed;
  }

  led_strip_.set_armed(state.armed);
}

void IO::ImuReceived(const ::sensor_msgs::Imu imu) {
  (void)imu;
  led_strip_.set_last_imu(::lib::phased_loop::GetCurrentTime());
}

// Pixhawk interfacing methods. ////////////////////////////////////////////////
// Send an arm command to the pixhawk when "arm" has a posedge.
void IO::PixhawkSendArm(bool arm) {
  // Only send arm command on posedge.
  if (arm == last_arm_) {
    return;
  }
  last_arm_ = arm;

  // If arm is false, don't arm.
  if (!arm) {
    return;
  }

  ::mavros_msgs::CommandBool cmd;
  cmd.request.value = true;

  if (arm_service_.call(cmd)) {
    ROS_INFO("Arm sent; got success %d", cmd.response.success);
  } else {
    ROS_ERROR("Arm failed!");
  }
}

// Send a takeoff command to the pixhawk when "takeoff" has a posedge.
void IO::PixhawkSendTakeoff(bool takeoff) {
  // Only send takeoff command on posedge.
  if (takeoff == last_takeoff_) {
    return;
  }
  last_takeoff_ = takeoff;

  // If takeoff is false, don't take off.
  if (!takeoff) {
    return;
  }

  ::mavros_msgs::SetMode cmd;
  cmd.request.base_mode = 0;
  cmd.request.custom_mode = kPixhawkCustomModeTakeoff;

  if (!set_mode_service_.call(cmd)) {
    ROS_INFO("Takeoff sent; got response %d", cmd.response.mode_sent);
  } else {
    ROS_ERROR("Takeoff failed!");
  }
}

// Send a land command to the pixhawk when "land" has a posedge.
void IO::PixhawkSendLand(bool land) {
  // Only send land command on posedge.
  if (land == last_land_) {
    return;
  }
  last_land_ = land;

  // If land is false, don't take off.
  if (!land) {
    return;
  }

  // Send land command to Pixhawk.
  ::mavros_msgs::SetMode cmd;
  cmd.request.base_mode = 0;
  cmd.request.custom_mode = kPixhawkCustomModeLand;

  if (!set_mode_service_.call(cmd)) {
    ROS_INFO("Land sent; got response %d", cmd.response.mode_sent);
  } else {
    ROS_ERROR("Land failed!");
  }
}

void IO::PixhawkSetGlobalPositionGoal(double latitude, double longitude,
                                      double altitude) {
  double current_time = ::lib::phased_loop::GetCurrentTime();

  // Don't send global position setpoint faster than a certain rate.
  if (current_time - last_global_position_setpoint_ <
      1.0 / kPixhawkGlobalSetpointMaxHz) {
    return;
  }
  last_global_position_setpoint_ = current_time;

  // Send global position setpoint to Pixhawk.
  ::mavros_msgs::GlobalPositionTarget target;
  target.header.stamp = ::ros::Time::now();
  target.latitude = latitude;
  target.longitude = longitude;
  target.altitude = altitude;
  target.yaw = 0;
  global_position_publisher_.publish(target);

  ROS_DEBUG("Sending global position setpoint: latitude(%f), longitude(%f), "
            "altitude(%f)",
            latitude, longitude, altitude);
}

} // namespace io
} // namespace controls
} // namespace src
