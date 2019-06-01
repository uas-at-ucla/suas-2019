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
    deployment_motor_setpoint_(0.0),
    gimbal_setpoint_(0.0),
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
    drone_program_subscriber_(ros_node_handle_.subscribe(
        "/uasatucla/proto/drone_program", kRosMessageQueueSize,
        &IO::DroneProgramReceived, this)),
    set_mode_service_(ros_node_handle_.serviceClient<::mavros_msgs::SetMode>(
        kRosSetModeService)),
    arm_service_(ros_node_handle_.serviceClient<::mavros_msgs::CommandBool>(
        kRosArmService)),
    takeoff_service_(ros_node_handle_.serviceClient<::mavros_msgs::CommandTOL>(
        kRosTakeoffService)),
    last_position_setpoint_(-std::numeric_limits<double>::infinity()),
    last_arm_state_(false),
    writer_thread_(&IO::WriterThread, this),
    writer_phased_loop_(kWriterPhasedLoopFrequency) {

  // Set up all actuators and send out initial outputs.
  InitializeActuators();

  // Chirp the alarm when the IO program starts.
  alarm_.AddAlert({kAlarmChirpDuration, 0});

  ROS_INFO("Flight loop initialized!");
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

#ifdef RASPI_DEPLOYMENT
  pigpio_stop(pigpio_);
#endif
}

void IO::WriterThread() {
  while (running_ && ::ros::ok()) {
    // Calculate alarm output.
    bool should_override_alarm =
        (should_override_alarm_ &&
         last_rc_in_ + kRcInTimeGap > ::ros::Time::now().toSec());
    bool should_alarm = alarm_.ShouldAlarm() || should_override_alarm;
<<<<<<< HEAD
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

      if (last_should_alarm != should_override_alarm_) {
        fly_start_time = ::lib::phased_loop::GetCurrentTime();
        did_trigger_takeoff = false;
        did_takeoff = false;
        did_finish_takeoff = false;
        did_offboard = false;
        did_mission = false;
        did_land = false;
        last_msg = ::lib::phased_loop::GetCurrentTime();
      }

      FlyToLocation();
    } else {
      if (last_should_alarm != should_override_alarm_) {
        if (did_offboard) {
          ::mavros_msgs::SetMode srv_setMode;
          srv_setMode.request.base_mode = 0;
          srv_setMode.request.custom_mode = "POSCTL";

          if (!set_mode_service_.call(srv_setMode)) {
            ROS_ERROR("Failed SetMode");
          }
        }
      }

      deployment_servo_setpoint_ = kDeploymentServoClosed;
    }

    last_should_alarm = should_override_alarm_;

    (void)gimbal_setpoint_;
#ifdef UAS_AT_UCLA_DEPLOYMENT
    // Write out alarm.
    digitalWrite(kAlarmGPIOPin, should_alarm ? HIGH : LOW);
=======
>>>>>>> 0f122b5e48a9062ce6411648672d69d0a1bb23cb

    // Calculate deployment output.
    ::lib::deployment::Input deployment_input;
    ::lib::deployment::Output deployment_output;
    deployment_input.direction = 0;
    deployment_.RunIteration(deployment_input, deployment_output);

    // Write out actuators.
    WriteAlarm(should_alarm);
    WriteGimbal(gimbal_setpoint_);
    WriteDeployment(deployment_output);

#ifndef RASPI_DEPLOYMENT
    PixhawkSetGlobalPositionGoal(34.173103, -118.482108, 100);
    // PixhawkSetGlobalPositionGoal(38.147483, -76.427778, 100);
#endif

    // Write output to LED strip.
    led_strip_.Render(false);

    // Write out sensors protobuf at a slower rate than the write loop.
    if (::ros::Time::now().toSec() > next_sensors_write_ &&
        ros_to_proto_.SensorsValid()) {
      sensors_publisher_.publish(ros_to_proto_.GetSensors());

      next_sensors_write_ =
          ::ros::Time::now().toSec() + kSensorsPublisherPeriod;
    }

    // Log the current GPIO outputs.
    ROS_DEBUG_STREAM_THROTTLE(
        0.1, "Writer thread iteration: "
                 << ::std::endl
                 << "alarm[" << should_alarm << "]" << ::std::endl
                 << "gimbal[" << gimbal_setpoint_ << "]" << ::std::endl
                 << "deployment_motor[" << deployment_output.motor << "]"
                 << ::std::endl
                 << "deployment_latch[" << deployment_output.latch << "]" << ::std::endl
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

////////////////////////////////////////////////////////////////////////////////
// ROS subscriber callbacks. ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// UAS@UCLA callbacks.

void IO::Output(const ::src::controls::Output output) {
  (void) output;
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

// Mavros callbacks.

void IO::RcInReceived(const ::mavros_msgs::RCIn rc_in) {
  bool new_should_override_alarm = should_override_alarm_;
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
<<<<<<< HEAD
  px4_mode_ = state.mode;

  if (state.armed != did_arm_) {
=======
  if (state.armed != last_arm_state_) {
>>>>>>> 0f122b5e48a9062ce6411648672d69d0a1bb23cb
    ROS_INFO_STREAM("Arming state changed: "
                    << (last_arm_state_ ? "ARMED" : "DISARMED") << " -> "
                    << (state.armed ? "ARMED" : "DISARMED"));
<<<<<<< HEAD
    did_arm_ = state.armed;
=======

    last_arm_state_ = state.armed;
>>>>>>> 0f122b5e48a9062ce6411648672d69d0a1bb23cb
  }

  led_strip_.set_armed(state.armed);
}

void IO::ImuReceived(const ::sensor_msgs::Imu imu) {
  // Silence unused variable warnings.
  (void)imu;

  led_strip_.set_last_imu(::ros::Time::now().toSec());
}

<<<<<<< HEAD
void IO::DroneProgramReceived(
    const ::src::controls::ground_controls::timeline::DroneProgram
        drone_program) {
  ::std::cout << "Executing Drone Program...\n";
  drone_program_ = drone_program;
  should_override_alarm_ = true;
  // ::std::cout << drone_program.DebugString() << "\n";
}

void IO::FlyToLocation() {
  if (!did_arm_) {
    ::std::cout << "arming!" << ::std::endl;
    ::mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if (arm_service_.call(srv)) {
      ROS_INFO("ARM send ok %d", srv.response.success);
    } else {
      ROS_ERROR("Failed arming or disarming");
    }
  } else if (!did_trigger_takeoff) {
    ::std::cout << "taking off!" << ::std::endl;
    ::mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "AUTO.TAKEOFF";
    if (set_mode_service_.call(srv_setMode)) {
      ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
    } else {
      ROS_ERROR("Failed SetMode");
    }
    did_trigger_takeoff = true;
  } else if (!did_takeoff) {
    if (px4_mode_ == "AUTO.TAKEOFF") {
      did_takeoff = true;
    }
  } else if (!did_finish_takeoff) {
    if (px4_mode_ == "AUTO.LOITER") {
      did_finish_takeoff = true;
    }
  } else if (!did_offboard) {
    if (px4_mode_ == "OFFBOARD") {
      did_offboard = true;
    } else {
      // setting setpoints is required before switching to offboard
      ::std::cout << "setting setpoint!" << ::std::endl;
      ::mavros_msgs::GlobalPositionTarget target;
      target.header.stamp = ::ros::Time::now();
      target.type_mask = ::mavros_msgs::GlobalPositionTarget::IGNORE_VX | ::mavros_msgs::GlobalPositionTarget::IGNORE_VY | ::mavros_msgs::GlobalPositionTarget::IGNORE_VZ | ::mavros_msgs::GlobalPositionTarget::IGNORE_AFX | ::mavros_msgs::GlobalPositionTarget::IGNORE_AFY | ::mavros_msgs::GlobalPositionTarget::IGNORE_AFZ | ::mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
      target.coordinate_frame = ::mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
      target.latitude = 38.145424243481784;
      target.longitude = -76.43065332806395;
      target.altitude = 50;
      target.yaw = 30;
      global_position_publisher_.publish(target);

      ::std::cout << "setting to offboard!" << ::std::endl;
      ::mavros_msgs::SetMode srv_setMode;
      srv_setMode.request.base_mode = 0;
      srv_setMode.request.custom_mode = "OFFBOARD";
      if (set_mode_service_.call(srv_setMode)) {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
      } else {
        ROS_ERROR("Failed SetMode");
      }
    }
  } else if (!did_mission) {
    ::std::cout << "setting setpoint!" << ::std::endl;
    ::mavros_msgs::GlobalPositionTarget target;
    target.header.stamp = ::ros::Time::now();
    target.type_mask = ::mavros_msgs::GlobalPositionTarget::IGNORE_VX | ::mavros_msgs::GlobalPositionTarget::IGNORE_VY | ::mavros_msgs::GlobalPositionTarget::IGNORE_VZ | ::mavros_msgs::GlobalPositionTarget::IGNORE_AFX | ::mavros_msgs::GlobalPositionTarget::IGNORE_AFY | ::mavros_msgs::GlobalPositionTarget::IGNORE_AFZ | ::mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
    target.coordinate_frame = ::mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    target.latitude = 38.145424243481784;
    target.longitude = -76.43065332806395;
    target.altitude = 50;
    target.yaw = 30;
    global_position_publisher_.publish(target);

    // did_mission = true; // when done with mission
  } else { // done with mission
    ::std::cout << "RTL!" << ::std::endl;
    ::mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "AUTO.RTL";
    if (set_mode_service_.call(srv_setMode)) {
      ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
=======
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
  gpio_write(pigpio_, kDeploymentMotorReverseGPIOPin, output.motor < 0);

  // Write hotwire.
  gpio_write(pigpio_, kDeploymentHotwireGPIOPin, output.hotwire);

  // Write latch.
  set_servo_pulsewidth(pigpio_, kDeploymentLatchServoGPIOPin,
                       output.latch ? kDeploymentServoClosed : kDeploymentServoOpen);
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
>>>>>>> 0f122b5e48a9062ce6411648672d69d0a1bb23cb
    } else {
      ROS_ERROR("Arm failed!");
    }
<<<<<<< HEAD
    did_land = true; // TODO
=======

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

// Send a global position setpoint to the Pixhawk, up to a maximum rate.
void IO::PixhawkSetGlobalPositionGoal(double latitude, double longitude,
                                      double altitude) {

  double current_time = ::ros::Time::now().toSec();

  // Don't send global position setpoint faster than a certain rate.
  if (current_time - last_position_setpoint_ <
      1.0 / kPixhawkGlobalSetpointMaxHz) {
    return;
>>>>>>> 0f122b5e48a9062ce6411648672d69d0a1bb23cb
  }
  last_position_setpoint_ = current_time;

  // Find WGS84 altitude to send as global position setpoint.
  // https://real.flightairmap.com/tools/geoid
  static GeographicLib::Geoid geoid_height_convert("egm96-5");
  double geoid_height = geoid_height_convert(latitude, longitude);

  // Send global position setpoint to Pixhawk, using the geoid height to convert
  // between WGS84 and AMSL altitudes.
  ::mavros_msgs::GlobalPositionTarget target;
  target.header.stamp = ::ros::Time::now();
  target.type_mask = ::mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                     ::mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                     ::mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                     ::mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                     ::mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                     ::mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                     ::mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
  target.coordinate_frame =
      ::mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
  target.latitude = latitude;
  target.longitude = longitude;
  target.altitude = altitude + geoid_height;
  target.yaw = 0;
  global_position_publisher_.publish(target);

  ROS_DEBUG("Sending global position setpoint: latitude(%f), longitude(%f), "
            "altitude(%f)",
            latitude, longitude, altitude);
}

} // namespace io
} // namespace controls
} // namespace src
