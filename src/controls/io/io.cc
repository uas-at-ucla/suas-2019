#include "io.h"

namespace src {
namespace controls {
namespace io {

// TODO(comran): Make trigger call a method in IO instead of a separate
// function.
void deploymentChannelOneTrigger(int gpio, int level, uint32_t tick) {
  (void)gpio;
  (void)level;
  (void)tick;
}

IO::IO() :
    alarm_(kWriterPhasedLoopFrequency),
    deployment_(kWriterPhasedLoopFrequency),
    next_sensors_write_(::ros::Time::now().toSec()),
    should_override_alarm_(false),
    last_rc_in_(::ros::Time::now().toSec()),
    gimbal_setpoint_(0.0),
    deployment_motor_setpoint_(0.0),
    latch_setpoint_(true),
    hotwire_setpoint_(false),
    deployment_manual_override_(false),
    latch_(true),
    deployment_motor_direction_(0),
    cut_line_(false),
    cancel_drop_(false),
    running_(true),
    ros_node_handle_(),
    sensors_publisher_(ros_node_handle_.advertise<::src::controls::Sensors>(
        kRosSensorsTopic, kRosMessageQueueSize)),
    global_position_publisher_(
        ros_node_handle_.advertise<::mavros_msgs::GlobalPositionTarget>(
            kRosGlobalPositionSetpointTopic, 10)),
    take_photo_publisher_(ros_node_handle_.advertise<std_msgs::String>(
        kRosTakePhotoTopic, kRosMessageQueueSize)),
    gimbal_publisher_(ros_node_handle_.advertise<::std_msgs::Float32>(
        kRosGimbalTopic, kRosMessageQueueSize, true)),
    deployment_motor_publisher_(ros_node_handle_.advertise<::std_msgs::Float32>(
        kRosDeploymentMotorTopic, kRosMessageQueueSize, true)),
    latch_publisher_(ros_node_handle_.advertise<::std_msgs::Bool>(
        kRosLatchTopic, kRosMessageQueueSize, true)),
    hotwire_publisher_(ros_node_handle_.advertise<::std_msgs::Bool>(
        kRosHotwireTopic, kRosMessageQueueSize, true)),
    gimbal_subscriber_(ros_node_handle_.subscribe(
        kRosGimbalTopic, kRosMessageQueueSize, &IO::GimbalSetpoint, this)),
    deployment_motor_subscriber_(ros_node_handle_.subscribe(
        kRosDeploymentMotorTopic, kRosMessageQueueSize,
        &IO::DeploymentMotorSetpoint, this)),
    latch_subscriber_(ros_node_handle_.subscribe(
        kRosLatchTopic, kRosMessageQueueSize, &IO::LatchSetpoint, this)),
    hotwire_subscriber_(ros_node_handle_.subscribe(
        kRosHotwireTopic, kRosMessageQueueSize, &IO::HotwireSetpoint, this)),
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
    // drone_program_subscriber_(
    //     ros_node_handle_.subscribe(kRosDroneProgramTopic,
    //     kRosMessageQueueSize,
    //                                &IO::DroneProgramReceived, this)),
    droppy_command_subscriber_(
        ros_node_handle_.subscribe(kRosMissionStatusTopic, kRosMessageQueueSize,
                                   &IO::DroppyCommandReceived, this)),
    set_mode_service_(ros_node_handle_.serviceClient<::mavros_msgs::SetMode>(
        kRosSetModeService)),
    arm_service_(ros_node_handle_.serviceClient<::mavros_msgs::CommandBool>(
        kRosArmService)),
    cmd_int_service_(ros_node_handle_.serviceClient<::mavros_msgs::CommandInt>(
        kRosCmdIntService)),
    takeoff_service_(ros_node_handle_.serviceClient<::mavros_msgs::CommandTOL>(
        kRosTakeoffService)),
    last_position_setpoint_(-std::numeric_limits<double>::infinity()),
    last_arm_state_(false),
    writer_thread_(&IO::WriterThread, this),
    writer_phased_loop_(kWriterPhasedLoopFrequency) {

  // Set up all actuators and send out initial outputs.
  InitializeActuators();
  ros_to_proto_.SetRunUasMission(false);
  ros_to_proto_.SetDoneDropping(false);

  ::std_msgs::Float32 init_gimbal;
  init_gimbal.data = gimbal_setpoint_;
  gimbal_publisher_.publish(init_gimbal);

  ::std_msgs::Float32 init_deployment_motor;
  init_deployment_motor.data = deployment_motor_setpoint_;
  deployment_motor_publisher_.publish(init_deployment_motor);

  ::std_msgs::Bool init_latch;
  init_latch.data = latch_setpoint_;
  latch_publisher_.publish(init_latch);

  ::std_msgs::Bool init_hotwire;
  init_hotwire.data = hotwire_setpoint_;
  hotwire_publisher_.publish(init_hotwire);

  // Chirp the alarm when the IO program starts.
  alarm_.AddAlert({kAlarmChirpDuration, 0});
}

void IO::Quit(int signal) {
  (void)signal;

  running_ = false;
  writer_thread_.join();

#ifdef RASPI_DEPLOYMENT
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

  ::std::cout << "KILL!\n";

#ifdef RASPI_DEPLOYMENT
  pigpio_stop(pigpio_);
#endif
}

void IO::WriterThread() {
  while (running_ && ::ros::ok()) {
    Sensors sensors;
    bool sensors_valid = ros_to_proto_.SensorsValid();
    if (sensors_valid) {
      sensors = ros_to_proto_.GetSensors();
    }

    // Calculate alarm output.
    bool should_override_alarm =
        (should_override_alarm_ &&
         last_rc_in_ + kRcInTimeGap > ::ros::Time::now().toSec());
    bool should_alarm = alarm_.ShouldAlarm() || should_override_alarm;

    // Overrides for simulation.
#ifndef RASPI_DEPLOYMENT
    static double start_time = ::ros::Time::now().toSec();
    ros_to_proto_.SetRunUasMission(::ros::Time::now().toSec() - start_time > 5);
#endif

    // Calculate deployment output.
    ::lib::deployment::Input deployment_input;
    ::lib::deployment::Output deployment_output;
    deployment_input.latch = latch_;
    deployment_input.direction = deployment_motor_direction_;
    deployment_input.cut = cut_line_;
    deployment_input.cancel = cancel_drop_;
    deployment_.RunIteration(deployment_input, deployment_output);

    // Record whether the deployment state machine is done dropping.
    ros_to_proto_.SetDoneDropping(deployment_output.end_drop);

    // Write out actuators.
    WriteAlarm(should_alarm);
    WriteGimbal(gimbal_setpoint_);
    WriteDeployment(deployment_output);

    // Take photos (test)
    TakePhotos();

    // Write output to LED strip.
    led_strip_.Render(false);

    // Write out sensors protobuf at a slower rate than the write loop.
    if (::ros::Time::now().toSec() > next_sensors_write_ && sensors_valid) {
      sensors_publisher_.publish(sensors);

      next_sensors_write_ =
          ::ros::Time::now().toSec() + kSensorsPublisherPeriod;
    }

    if (sensors_valid && SafetyChecks(sensors) && ros_to_proto_.OutputValid() &&
        sensors.run_uas_mission()) {
      ::src::controls::Output output = ros_to_proto_.GetOutput();

      // Only listen to output if safety pilot override is not active.
      PixhawkSendModePosedge(kPixhawkCustomModeTakeoff,
                             output.trigger_takeoff());
      PixhawkSendModePosedge(kPixhawkCustomModeLoiter, output.trigger_hold());
      PixhawkSendModePosedge(kPixhawkCustomModeOffboard,
                             output.trigger_offboard());
      PixhawkSendModePosedge(kPixhawkCustomModeReturnToLand,
                             output.trigger_rtl());
      PixhawkSendModePosedge(kPixhawkCustomModeLand, output.trigger_land());
      PixhawkSendModePosedge(kPixhawkArmCommand, output.trigger_arm());

      if (output.send_setpoint()) {
        PixhawkSetGlobalPositionGoal(
            output.setpoint_latitude(), output.setpoint_longitude(),
            output.setpoint_altitude(), output.setpoint_yaw());
      }
    }

    // Log the current GPIO outputs.
    ROS_DEBUG_STREAM_THROTTLE(
        1.0 / kActuatorLogHz,
        "Writer thread iteration: "
            << ::std::endl
            << "alarm[" << should_alarm << "]" << ::std::endl
            << "gimbal[" << gimbal_setpoint_ << "]" << ::std::endl
            << "deployment_motor[" << deployment_output.motor << "]"
            << ::std::endl
            << "deployment_latch[" << deployment_output.latch << "]"
            << ::std::endl
            << "deployment_hotwire[" << deployment_output.hotwire << "]"
            << ::std::endl
#ifdef LOG_LED_STRIP
            << "led_strip[" << led_strip_.GetStrip() << ::std::endl
#endif
            << "]");

    // Wait until next iteration of loop.
    writer_phased_loop_.sleep();
  }
}

bool IO::SafetyChecks(Sensors &sensors) {
  // No checks if disarmed.
  if (!sensors.armed()) {
    return true;
  }

  double current_time = ::ros::Time::now().toSec();
  ::std::cout << current_time - last_rc_in_ << ::std::endl;

  bool everythings_fine = true;

  if (current_time - last_rc_in_ > kRcLossAlertTimeout) {
    everythings_fine = false;
  }

  // Alarm to let us know that RC was lost for 5 seconds while armed.
  // static bool last_alarm_rc_loss = false;
  // if(current_time - last_rc_in_ > kRcLossAlertTimeout) {
  //   if(!last_alarm_rc_loss) {
  //     alarm_.AddAlert({0.2, 0.1});
  //   }
  //   last_alarm_rc_loss = true;
  // } else {
  //   last_alarm_rc_loss = false;
  // }

  // RTL if RC is lost for a certain amount of time.
  // static bool last_lost_rc_rtl = false;
  // if(current_time - last_rc_in_ > kRcLossRtlTimeout) {
  //   ROS_ERROR("Lost RC! Performing RTL...");

  //   if(!last_lost_rc_rtl) {
  //     alarm_.AddAlert({0.1, 0.1});
  //   }
  //   last_lost_rc_rtl = true;

  //   bool rtl_trigger = ::std::fmod(sensors.time() - last_rc_in_,
  //   kTriggerPeriod) >
  //                     kTriggerPeriod / 2.0 && sensors.autopilot_state() !=
  //                     kPixhawkCustomModeReturnToLand;
  //   PixhawkSendModePosedge(kPixhawkCustomModeReturnToLand, rtl_trigger);
  //   everythings_fine = false;
  // } else {
  //   last_lost_rc_rtl = false;
  // }

  // // Flight term if RC is lost for a certain amount of time.
  // static bool last_lost_rc_term = false;
  // if(current_time - last_rc_in_ > kRcLossTerminateTimeout) {
  //   ROS_ERROR("Lost RC! Performing Term...");

  //   if(!last_lost_rc_term) {
  //     alarm_.AddAlert({0.1, 0.1});
  //   }
  //   last_lost_rc_term = true;

  //   bool term_trigger = ::std::fmod(sensors.time() - last_rc_in_,
  //   kTriggerPeriod) >
  //                     kTriggerPeriod / 2.0;
  //   ::std::cout << "term trig: " << term_trigger << ::std::endl;
  //   PixhawkSendModePosedge(kPixhawkFlightTermCommand, term_trigger);
  //   everythings_fine = false;
  // } else {
  //   last_lost_rc_term = false;
  // }

  return everythings_fine;
}

////////////////////////////////////////////////////////////////////////////////
// ROS subscriber callbacks. ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// UAS@UCLA callbacks.

void IO::GimbalSetpoint(const ::std_msgs::Float32 gimbal_setpoint) {
  gimbal_setpoint_ = gimbal_setpoint.data;
}

void IO::DeploymentMotorSetpoint(
    const ::std_msgs::Float32 deployment_motor_setpoint) {
  deployment_motor_setpoint_ = deployment_motor_setpoint.data;
}

void IO::LatchSetpoint(const ::std_msgs::Bool latch_setpoint) {
  latch_setpoint_ = latch_setpoint.data;
}

void IO::HotwireSetpoint(const ::std_msgs::Bool hotwire_setpoint) {
  hotwire_setpoint_ = hotwire_setpoint.data;
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

// Mavros callbacks.

void IO::RcInReceived(const ::mavros_msgs::RCIn rc_in) {
  bool new_should_override_alarm = should_override_alarm_;

  if (rc_in.channels[kThrottleRcChannel - 1] < kRcInMinimumThreshold) {
    return;
  }

  last_rc_in_ = ::ros::Time::now().toSec();

  // Trigger the alarm if the RC controller override switch was flipped.
  if (rc_in.channels[kAlarmOverrideRcChannel - 1] >
      kAlarmOverrideRcSignalThreshold) {
    new_should_override_alarm = true;
  } else {
    new_should_override_alarm = false;
  }

  int deployment_rc_in = rc_in.channels[kDeploymentMotorRcChannel - 1];
  if (deployment_rc_in > 900) {
    ::std_msgs::Float32 deployment_motor_setpoint;
    deployment_motor_setpoint.data =
        ::std::max(::std::min((deployment_rc_in - 1500) / 500.0, 1.0), -1.0);
    if (::std::abs(deployment_motor_setpoint.data) < 0.1) {
      deployment_motor_setpoint.data = 0;
    }
    deployment_motor_publisher_.publish(deployment_motor_setpoint);
    deployment_manual_override_ =
        true; // if we override deployment with rc, set this to true to allow
              // manual input from groundstation
  }           /*else {
              deployment_motor_setpoint.data = 0;
            }*/

  bool uas_mission_run_current =
      rc_in.channels[kUasMissionRcChannel - 1] > 1700;
  if (ros_to_proto_.SensorsValid()) {
    bool uas_mission_run_last = ros_to_proto_.GetSensors().run_uas_mission();
    if (uas_mission_run_current != uas_mission_run_last) {
      ROS_INFO("Allow UAS mission changed: %d -> %d", uas_mission_run_last,
               uas_mission_run_current);
    }
  }
  ros_to_proto_.SetRunUasMission(uas_mission_run_current);

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
  // Silence unused variable warnings.
  (void)imu;

  led_strip_.set_last_imu(::ros::Time::now().toSec());
}

void IO::DroppyCommandReceived(const ::std_msgs::String droppy_command) {
  if (droppy_command.data == "START_DROP") {
    ROS_INFO_THROTTLE(1, "Initiating UGV Drop");
    latch_ = false;
    deployment_motor_direction_ = 1;
  } else if (droppy_command.data == "CUT_LINE") {
    ROS_INFO_THROTTLE(1, "Cutting Fishing Line");
    deployment_motor_direction_ = 0;
    cut_line_ = true;
  } else if (droppy_command.data == "MOTOR_UP") {
    ROS_INFO_THROTTLE(1, "Raising UGV");
    deployment_motor_direction_ = -1;
  } else if (droppy_command.data == "MOTOR_DOWN") {
    ROS_INFO_THROTTLE(1, "Lowering UGV");
    deployment_motor_direction_ = 1;
  } else if (droppy_command.data == "MOTOR_STOP") {
    ROS_INFO_THROTTLE(1, "Stop lowering UGV");
    deployment_motor_direction_ = 0;
  } else if (droppy_command.data == "CANCEL_DROP") {
    ROS_INFO_THROTTLE(1, "Cancel drop");
    cancel_drop_ = true;
  } else if (droppy_command.data == "RESET_LATCH") {
    ROS_INFO_THROTTLE(1, "Resetting Latch");
    latch_ = true;
    cancel_drop_ = false;
  } else if (droppy_command.data == "STOP_CUT") {
    ROS_INFO_THROTTLE(1, "Stop Hotwire");
    cut_line_ = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Actuator output write methods. //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void IO::InitializeActuators() {
#ifdef RASPI_DEPLOYMENT
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
}

void IO::WriteAlarm(bool alarm) {
#ifdef RASPI_DEPLOYMENT
  // Write alarm.
  digitalWrite(kAlarmGPIOPin, alarm ? HIGH : LOW);
#else
  // Silence unused variable warnings.
  (void)alarm;
#endif
}

void IO::WriteGimbal(double pitch) {
#ifdef RASPI_DEPLOYMENT
  // Cap pitch range to [-1, 1].
  pitch = ::std::max(::std::min(pitch, 1.0), -1.0);

  // Write gimbal pitch.
  set_servo_pulsewidth(pigpio_, kGimbalGPIOPin, 1500 + pitch * 500);
#else
  // Silence unused variable warnings.
  (void)pitch;
#endif
}

void IO::WriteDeployment(::lib::deployment::Output &output) {
#ifdef RASPI_DEPLOYMENT
  // Cap motor range to [-1, 1].
  output.motor = ::std::max(::std::min(output.motor, 1.0), -1.0);

  // Write motor.
  set_PWM_dutycycle(pigpio_, kDeploymentMotorGPIOPin,
                    output.motor * (output.motor >= 0 ? 1 : -1) * 100);
  gpio_write(pigpio_, kDeploymentMotorReverseGPIOPin, output.motor > 0);

  // Write hotwire.
  gpio_write(pigpio_, kDeploymentHotwireGPIOPin, output.hotwire);

  // Write latch.
  set_servo_pulsewidth(pigpio_, kDeploymentLatchServoGPIOPin,
                       output.latch ? kDeploymentServoClosed
                                    : kDeploymentServoOpen);
#else
  // Silence unused variable warnings.
  (void)output;
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Pixhawk interfacing methods. ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Send a SetMode command to the Pixhawk when the signal has a posedge.
void IO::PixhawkSendModePosedge(::std::string mode, bool signal) {
  // Only send takeoff command on posedge.
  if (last_mode_signals_.count(mode) > 0 &&
      signal == last_mode_signals_[mode]) {
    return;
  }
  last_mode_signals_[mode] = signal;

  // If signal is false, don't take off.
  if (!signal) {
    return;
  }

  // Handle case where an arm command is provided.
  if (mode == kPixhawkArmCommand) {
    // Send arm command to Pixhawk.
    ::mavros_msgs::CommandBool cmd;
    cmd.request.value = true;

    if (arm_service_.call(cmd)) {
      ROS_INFO("Arm sent; got success %d", cmd.response.success);
    } else {
      ROS_ERROR("Arm failed!");
    }

    return;
  }

  // Handle case where a flight term command is provided.
  if (mode == kPixhawkFlightTermCommand) {
    // Send arm command to Pixhawk.
    ::mavros_msgs::CommandInt cmd;
    cmd.request.command = 185;
    cmd.request.param1 = 1;
    cmd.request.param2 = 0;
    cmd.request.param3 = 0;
    cmd.request.param4 = 0;
    cmd.request.x = 0;
    cmd.request.y = 0;
    cmd.request.z = 0;
    ::std::cout << "sending!\n";

    if (cmd_int_service_.call(cmd)) {
      ROS_INFO("Flight Term sent; got success %d", cmd.response.success);
    } else {
      ROS_ERROR("Flight Term failed!");
    }

    return;
  }

  // Send SetMode command to Pixhawk.
  ::mavros_msgs::SetMode cmd;
  cmd.request.base_mode = 0;
  cmd.request.custom_mode = mode;

  if (!set_mode_service_.call(cmd)) {
    ROS_INFO("%s sent; got response %d", mode.c_str(), cmd.response.mode_sent);
  } else {
    ROS_ERROR("%s failed!", mode.c_str());
  }
}

// Send a global position setpoint to the Pixhawk, up to a maximum rate. Takes
// in latitude, longitude, altitude (relative to home position), and yaw
// (in degrees, relative to north CW).
void IO::PixhawkSetGlobalPositionGoal(double latitude, double longitude,
                                      double altitude, double yaw) {

  double current_time = ::ros::Time::now().toSec();

  // Don't send global position setpoint faster than a certain rate.
  if (current_time - last_position_setpoint_ <
      1.0 / kPixhawkGlobalSetpointMaxHz) {
    return;
  }
  last_position_setpoint_ = current_time;

  // Assert that we have the latest sensors protobuf before continuing.
  if (!ros_to_proto_.SensorsValid()) {
    ROS_ERROR("Attempted to send global position goal without valid Sensors!");
    return;
  }

  // Base all altitude setpoints off home altitude.
  double home_altitude = ros_to_proto_.GetSensors().home_altitude();
  double yaw_radians = ::std::fmod(yaw - 90.0, 360.0) * -M_PI / 180.0;

  /*
  // Find WGS84 altitude to send as global position setpoint.
  // https://real.flightairmap.com/tools/geoid
  static GeographicLib::Geoid geoid_height_convert("egm96-5");
  double geoid_height = geoid_height_convert(latitude, longitude);
  */

  // Send global position setpoint to Pixhawk, using the geoid height to convert
  // between WGS84 and AMSL altitudes.
  ::mavros_msgs::GlobalPositionTarget target;
  target.header.stamp = ::ros::Time::now();
  target.latitude = latitude;
  target.longitude = longitude;
  target.altitude = home_altitude + altitude;
  target.yaw = yaw_radians;
  global_position_publisher_.publish(target);

  ROS_DEBUG_THROTTLE(
      1.0 / kActuatorLogHz,
      "Sending global position setpoint: latitude(%f), longitude(%f), "
      "altitude(%f)",
      latitude, longitude, altitude);
}

///////////////////////////////////////////////////////////////////////////
// Take photos. ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void IO::TakePhotos() {
  std_msgs::String ret;
  ret.data = "TRUE";
  take_photo_publisher_.publish(ret);
}

} // namespace io
} // namespace controls
} // namespace src
