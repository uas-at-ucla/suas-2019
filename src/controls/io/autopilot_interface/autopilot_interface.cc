#include "autopilot_interface.h"

namespace src {
namespace controls {
namespace io {
namespace autopilot_interface {

AutopilotInterface::AutopilotInterface(const char *address) :
    reading_status_(0),
    writing_status_(0),
    write_count_(0),
    time_to_exit_(false) {
  system_id = 1;
  autopilot_id = 1;
  companion_id = 3;

  current_messages.sysid = system_id;
  current_messages.compid = autopilot_id;

  char udp[30];
  strcpy(udp, "udp://");
  strcat(udp, address);
  strcat(udp, ":8084@:8084");

  pixhawk_ = ::mavconn::MAVConnInterface::open_url(udp, 0, 0);
  pixhawk_->set_protocol_version(mavconn::Protocol::V20);
  /*
     pixhawk_->message_received_cb = [this](const mavlink_message_t *msg,
                                         const ::mavconn::Framing framing) {
     publish_message(msg, framing);
    }
    */
  void (*publishPtr)(const mavlink_message_t, const ::mavconn::Framing) = 
     ros_publisher_.publish_message;
  pixhawk_->message_received_cb = publishPtr;
}

AutopilotInterface::~AutopilotInterface() {}

uint64_t AutopilotInterface::get_time_usec() {
  static struct timeval _time_stamp;
  gettimeofday(&_time_stamp, NULL);
  return _time_stamp.tv_sec * 1e6 + _time_stamp.tv_usec;
}

void AutopilotInterface::set_position(float x, float y, float z,
                  mavlink_set_position_target_local_ned_t &sp) {
  sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

  sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

  sp.x = x;
  sp.y = y;
  sp.z = z;
}

void AutopilotInterface::set_velocity(float vx, float vy, float vz,
                  mavlink_set_position_target_local_ned_t &sp) {
  sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

  sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

  sp.vx = vx;
  sp.vy = vy;
  sp.vz = vz;
}

void AutopilotInterface::set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp) {
  sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

  sp.yaw = yaw;
}

void AutopilotInterface::set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp) {
  sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

  sp.yaw_rate = yaw_rate;
}

/*
AutopilotInterface::AutopilotInterface(const char *address) :
    write_tid_(0),
    reading_status_(0),
    writing_status_(0),
    write_count_(0),
    time_to_exit_(false) {
  system_id = 1;
  autopilot_id = 1;
  companion_id = 3;

  current_messages.sysid = system_id;
  current_messages.compid = autopilot_id;

  char udp[30];
  strcpy(udp, "udp://");
  strcat(udp, address);
  strcat(udp, ":8084@:8084");

  pixhawk_ = ::mavconn::MAVConnInterface::open_url(udp, 0, 0);
  pixhawk_->set_protocol_version(mavconn::Protocol::V20);
  pixhawk_->message_received_cb =
      ::std::bind(&ros_publisher::RosPublisher::WriteMessage, &ros_publisher_,
                  ::std::placeholders::_1, ::std::placeholders::_2);
}
*/

void AutopilotInterface::update_setpoint(
    mavlink_set_position_target_local_ned_t setpoint) {
  current_setpoint = setpoint;
}

void AutopilotInterface::write_message(mavlink_message_t message) {
  pixhawk_->send_message(&message);
}

void AutopilotInterface::write_setpoint() {
  // pull from position target
  mavlink_set_position_target_local_ned_t sp = current_setpoint;

  // double check some system parameters
  if (not sp.time_boot_ms)
    sp.time_boot_ms = (uint32_t)(get_time_usec() / 1e3);
  sp.target_system = system_id;
  sp.target_component = autopilot_id;

  mavlink_message_t message;
  mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id,
                                                   &message, &sp);

  write_message(message);
}

void AutopilotInterface::start() {
  // Component ID
  if (not autopilot_id) {
    autopilot_id = current_messages.compid;
  }

  // Wait for initial position ned
  while (not(current_messages.time_stamps.local_position_ned &&
             current_messages.time_stamps.attitude)) {
    if (time_to_exit_) {
      return;
    }
    usleep(1e6 / 2);
  }

  // copy initial position ned
  Mavlink_Messages local_data = current_messages;
  initial_position.x = local_data.local_position_ned.x;
  initial_position.y = local_data.local_position_ned.y;
  initial_position.z = local_data.local_position_ned.z;
  initial_position.vx = local_data.local_position_ned.vx;
  initial_position.vy = local_data.local_position_ned.vy;
  initial_position.vz = local_data.local_position_ned.vz;
  initial_position.yaw = local_data.attitude.yaw;
  initial_position.yaw_rate = local_data.attitude.yawspeed;

  thread_ = ::std::thread(&AutopilotInterface::start_write_thread, this);

  // Wait for write thread to be started.
  while (not writing_status_)
    usleep(1e6 / 10);

  return;
}

void AutopilotInterface::Arm() {
  mavlink_command_long_t com;
  com.target_system = system_id;
  com.target_component = autopilot_id;
  com.command = MAV_CMD_COMPONENT_ARM_DISARM;
  com.confirmation = true;
  com.param1 = 1; // Should arm.

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  write_message(message);
}

void AutopilotInterface::Disarm() {
  mavlink_command_long_t com;
  com.target_system = system_id;
  com.target_component = autopilot_id;
  com.command = MAV_CMD_COMPONENT_ARM_DISARM;
  com.confirmation = true;
  com.param1 = 0; // Should disarm.

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  write_message(message);
}

void AutopilotInterface::DoGimbal() {
  static float position = 0;
  static bool flip = false;
  if (position > 89.9) {
    flip = true;
  } else if (position < 0.1) {
    flip = false;
  }

  if (flip) {
    position -= 10;
  } else {
    position += 10;
  }

  {
    mavlink_command_long_t com;
    com.target_system = system_id;
    com.target_component = autopilot_id;
    com.command = MAV_CMD_DO_MOUNT_CONFIGURE;
    com.param1 = MAV_MOUNT_MODE_MAVLINK_TARGETING;
    com.param2 = 1;
    com.param3 = 1;
    com.param4 = 1;
    com.param5 = 0;
    com.param6 = 0;
    com.param7 = 0;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    write_message(message);
  }

  {
    mavlink_command_long_t com;
    com.target_system = system_id;
    com.target_component = autopilot_id;
    com.command = MAV_CMD_DO_MOUNT_CONTROL;
    com.param1 = position;
    com.param2 = position;
    com.param3 = position;
    com.param4 = 0;
    com.param5 = 0;
    com.param6 = 0;
    com.param7 = MAV_MOUNT_MODE_MAVLINK_TARGETING;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    write_message(message);
  }
}

void AutopilotInterface::Takeoff() {
  mavlink_global_position_int_t gps = current_messages.global_position_int;

  mavlink_command_long_t com;
  com.target_system = system_id;
  com.target_component = autopilot_id;
  com.command = MAV_CMD_NAV_TAKEOFF;
  com.confirmation = true;
  com.param5 = static_cast<double>(gps.lat) / 1e7;
  com.param6 = static_cast<double>(gps.lon) / 1e7;
  com.param7 = static_cast<double>(gps.alt) / 1e3;

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  write_message(message);
}

void AutopilotInterface::Hold() {
  mavlink_command_long_t com;
  com.target_system = system_id;
  com.target_component = autopilot_id;
  com.command = MAV_CMD_DO_SET_MODE;
  com.confirmation = true;
  com.param1 = MAV_MODE_AUTO_ARMED;
  com.param2 = 4;
  com.param3 = 3;

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  write_message(message);
}

void AutopilotInterface::ReturnToLaunch() {
  mavlink_command_long_t com;
  com.target_system = system_id;
  com.target_component = autopilot_id;
  com.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
  com.confirmation = true;

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  write_message(message);
}

void AutopilotInterface::Offboard() {
  mavlink_command_long_t com = {0};
  com.target_system = system_id;
  com.target_component = autopilot_id;
  com.command = MAV_CMD_NAV_GUIDED_ENABLE;
  com.confirmation = true;
  com.param1 = 1;

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  write_message(message);
}

void AutopilotInterface::Land() {
  mavlink_global_position_int_t gps = current_messages.global_position_int;

  mavlink_command_long_t com;
  com.target_system = system_id;
  com.target_component = autopilot_id;
  com.command = MAV_CMD_NAV_LAND;
  com.confirmation = true;
  com.param5 = static_cast<double>(gps.lat) / 1e7;
  com.param6 = static_cast<double>(gps.lon) / 1e7;

  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  write_message(message);
}

void AutopilotInterface::FlightTermination() {
  // NO LONGER DOES ANYTHING!
  // Disarm();

  // mavlink_command_long_t com;
  // com.target_system = system_id;
  // com.target_component = autopilot_id;
  // com.command = MAV_CMD_DO_FLIGHTTERMINATION;
  // com.confirmation = true;
  // com.param1 = 1;

  // mavlink_message_t message;
  // mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  // write_message(message);
}

void AutopilotInterface::set_message_period() {
  for (int i = 0; i < 255; i++) {
    int32_t interval = -1;
    bool valid = false;

    switch (i) {
      case MAVLINK_MSG_ID_HEARTBEAT:
      case MAVLINK_MSG_ID_SYS_STATUS:
      case MAVLINK_MSG_ID_BATTERY_STATUS:
      case MAVLINK_MSG_ID_RADIO_STATUS:
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      case MAVLINK_MSG_ID_HIGHRES_IMU:
      case MAVLINK_MSG_ID_ATTITUDE:
        interval = 1e6 / 25;
        valid = true;
        break;
    }

    if (valid) {
      mavlink_command_long_t com;
      com.target_system = system_id;
      com.target_component = autopilot_id;
      com.command = MAV_CMD_SET_MESSAGE_INTERVAL;
      com.param1 = i;
      com.param2 = interval;

      mavlink_message_t message;
      mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

      write_message(message);
    }
  }
}

void AutopilotInterface::set_params() {
  set_param("MIS_TAKEOFF_ALT", 5.0);
  set_param("MPC_XY_CRUISE  ", 10.0);
  set_param("MPC_XY_VEL_MAX ", 10.0);
  set_param("MC_YAWRAUTO_MAX", 25.0);
}

void AutopilotInterface::set_param(const char id[], float value) {
  mavlink_param_set_t param_config;
  param_config.target_system = system_id;
  param_config.target_component = autopilot_id;

  memcpy(param_config.param_id, id, ::std::min(16, (int)strlen(id)));
  param_config.param_value = value;

  mavlink_message_t message;
  mavlink_msg_param_set_encode(system_id, companion_id, &message,
                               &param_config);

  write_message(message);
}

void AutopilotInterface::stop() {
  time_to_exit_ = true;
  // pixhawk_->close();

  //pthread_join(write_tid_, NULL);
}

void AutopilotInterface::start_write_thread(void) {
  if (not writing_status_ == false) {
    fprintf(stderr, "write thread already running\n");
    return;
  }

  else {
    write_thread();
    return;
  }
}

void AutopilotInterface::handle_quit(int sig) {
  try {
    stop();
  } catch (int error) {
    fprintf(stderr, "Warning, could not stop autopilot interface\n");
  }
}

void AutopilotInterface::write_thread(void) {
  writing_status_ = 2;

  // Send initial stationary command.
  mavlink_set_position_target_local_ned_t sp;
  sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
                 MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
  sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
  sp.vx = 0.0;
  sp.vy = 0.0;
  sp.vz = 0.0;
  sp.yaw_rate = 0.0;

  current_setpoint = sp;

  write_setpoint();
  writing_status_ = true;

  set_message_period();
  set_params();

  while (!time_to_exit_) {
    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe.

    usleep(1e6 / 4);
    write_setpoint();
  }

  writing_status_ = false;

  return;
}

void ROSPublisher::publish_message(const mavlink_message_t *msg, const ::mavconn::Framing framing) {
    switch (msg->msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        mavlink_msg_heartbeat_decode(msg, &(current_messages.heartbeat));
        current_messages.time_stamps.heartbeat = get_time_usec();
        break;

      case MAVLINK_MSG_ID_SYS_STATUS:
        mavlink_msg_sys_status_decode(msg, &(current_messages.sys_status));
        current_messages.time_stamps.sys_status = get_time_usec();
        break;

      case MAVLINK_MSG_ID_BATTERY_STATUS:
        mavlink_msg_battery_status_decode(msg,
                                          &(current_messages.battery_status));
        current_messages.time_stamps.battery_status = get_time_usec();
        break;

      case MAVLINK_MSG_ID_RADIO_STATUS:
        mavlink_msg_radio_status_decode(msg, &(current_messages.radio_status));
        current_messages.time_stamps.radio_status = get_time_usec();
        break;

      case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        mavlink_msg_local_position_ned_decode(
            msg, &(current_messages.local_position_ned));
        current_messages.time_stamps.local_position_ned = get_time_usec();
        break;

      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        mavlink_msg_global_position_int_decode(
            msg, &(current_messages.global_position_int));
        current_messages.time_stamps.global_position_int = get_time_usec();
        break;

      case MAVLINK_MSG_ID_GPS_RAW_INT:
        mavlink_msg_gps_raw_int_decode(msg, &(current_messages.gps_raw_int));
        current_messages.time_stamps.gps_raw_int = get_time_usec();
        break;

      case MAVLINK_MSG_ID_HIGHRES_IMU:
        mavlink_msg_highres_imu_decode(msg, &(current_messages.highres_imu));
        current_messages.time_stamps.highres_imu = get_time_usec();
        break;

      case MAVLINK_MSG_ID_ATTITUDE:
        mavlink_msg_attitude_decode(msg, &(current_messages.attitude));
        current_messages.time_stamps.attitude = get_time_usec();
        break;

      case MAVLINK_MSG_ID_VFR_HUD:
        mavlink_msg_vfr_hud_decode(msg, &(current_messages.vfr_hud));
        current_messages.time_stamps.vfr_hud = get_time_usec();
        break;

      case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
        mavlink_msg_actuator_control_target_decode(
            msg, &(current_messages.control_target));

        break;

      default:
        break;

}

} // namespace autopilot_interface
} // namespace io
} // namespace controls
} // namespace src
