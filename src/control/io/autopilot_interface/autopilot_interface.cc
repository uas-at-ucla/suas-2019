#include "autopilot_interface.h"

namespace spinny {
namespace control {
namespace io {
namespace autopilot_interface {

uint64_t get_time_usec() {
  static struct timeval _time_stamp;
  gettimeofday(&_time_stamp, NULL);
  return _time_stamp.tv_sec * 1e6 + _time_stamp.tv_usec;
}

void set_position(float x, float y, float z,
                  mavlink_set_position_target_local_ned_t &sp) {
  sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

  sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

  sp.x = x;
  sp.y = y;
  sp.z = z;

  printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);
}

void set_velocity(float vx, float vy, float vz,
                  mavlink_set_position_target_local_ned_t &sp) {
  sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

  sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

  sp.vx = vx;
  sp.vy = vy;
  sp.vz = vz;

  printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy,
         sp.vz);
}

void set_acceleration(float ax, float ay, float az,
                      mavlink_set_position_target_local_ned_t &sp) {
  // NOT IMPLEMENTED
  fprintf(stderr, "set_acceleration doesn't work yet \n");
  throw 1;

  sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
                 MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

  sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

  sp.afx = ax;
  sp.afy = ay;
  sp.afz = az;
}

void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp) {
  sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

  sp.yaw = yaw;

  printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);
}

void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp) {
  sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

  sp.yaw_rate = yaw_rate;
}

AutopilotInterface::AutopilotInterface(const char *serial_port, int baud)
    : read_tid_(0),
      write_tid_(0),
      reading_status_(0),
      writing_status_(0),
      control_status_(0),
      write_count_(0),
      time_to_exit_(false) {

  system_id = 0;
  autopilot_id = 0;
  companion_id = 0;

  current_messages.sysid = system_id;
  current_messages.compid = autopilot_id;

  mavlink_serial_ = new MavlinkSerial(serial_port, baud);
}

AutopilotInterface::~AutopilotInterface() {}

void AutopilotInterface::update_setpoint(
    mavlink_set_position_target_local_ned_t setpoint) {
  current_setpoint = setpoint;
}

void AutopilotInterface::read_messages() {
  bool success;
  bool received_all = false;
  TimeStamps this_timestamps;

  // Blocking wait for new data
  while (!received_all and !time_to_exit_) {
    mavlink_message_t message;
    success = mavlink_serial_->read_message(message);

    if (success) {
      // Store message sysid and compid.
      // Note this doesn't handle multiple message sources.
      current_messages.sysid = message.sysid;
      current_messages.compid = message.compid;

      // Handle Message ID
      switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
          current_messages.time_stamps.heartbeat = get_time_usec();
          this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
          break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS: {
          mavlink_msg_sys_status_decode(&message,
                                        &(current_messages.sys_status));
          current_messages.time_stamps.sys_status = get_time_usec();
          this_timestamps.sys_status = current_messages.time_stamps.sys_status;
          break;
        }

        case MAVLINK_MSG_ID_BATTERY_STATUS: {
          mavlink_msg_battery_status_decode(&message,
                                            &(current_messages.battery_status));
          current_messages.time_stamps.battery_status = get_time_usec();
          this_timestamps.battery_status =
              current_messages.time_stamps.battery_status;
          break;
        }

        case MAVLINK_MSG_ID_RADIO_STATUS: {
          mavlink_msg_radio_status_decode(&message,
                                          &(current_messages.radio_status));
          current_messages.time_stamps.radio_status = get_time_usec();
          this_timestamps.radio_status =
              current_messages.time_stamps.radio_status;
          break;
        }

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
          mavlink_msg_local_position_ned_decode(
              &message, &(current_messages.local_position_ned));
          current_messages.time_stamps.local_position_ned = get_time_usec();
          this_timestamps.local_position_ned =
              current_messages.time_stamps.local_position_ned;
          break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          mavlink_msg_global_position_int_decode(
              &message, &(current_messages.global_position_int));
          current_messages.time_stamps.global_position_int = get_time_usec();
          this_timestamps.global_position_int =
              current_messages.time_stamps.global_position_int;
          break;
        }

        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: {
          mavlink_msg_position_target_local_ned_decode(
              &message, &(current_messages.position_target_local_ned));
          current_messages.time_stamps.position_target_local_ned =
              get_time_usec();
          this_timestamps.position_target_local_ned =
              current_messages.time_stamps.position_target_local_ned;
          break;
        }

        case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: {
          mavlink_msg_position_target_global_int_decode(
              &message, &(current_messages.position_target_global_int));
          current_messages.time_stamps.position_target_global_int =
              get_time_usec();
          this_timestamps.position_target_global_int =
              current_messages.time_stamps.position_target_global_int;
          break;
        }

        case MAVLINK_MSG_ID_HIGHRES_IMU: {
          mavlink_msg_highres_imu_decode(&message,
                                         &(current_messages.highres_imu));
          current_messages.time_stamps.highres_imu = get_time_usec();
          this_timestamps.highres_imu =
              current_messages.time_stamps.highres_imu;
          break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
          mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
          current_messages.time_stamps.attitude = get_time_usec();
          this_timestamps.attitude = current_messages.time_stamps.attitude;
          break;
        }

        default: { break; }
      }
    }

    // Check for receipt of all items
    received_all = this_timestamps.heartbeat &&
                   //        this_timestamps.battery_status &&
                   //        this_timestamps.radio_status &&
                   //        this_timestamps.local_position_ned &&
                   //        this_timestamps.global_position_int &&
                   //        this_timestamps.position_target_local_ned
                   //&&
                   //        this_timestamps.position_target_global_int
                   //&&
                   //        this_timestamps.highres_imu &&
                   //        this_timestamps.attitude &&
                   this_timestamps.sys_status;

    // give the write thread time to use the port
    if (writing_status_ > false) {
      usleep(100);  // look for components of batches at 10kHz
    }
  }
}

int AutopilotInterface::write_message(mavlink_message_t message) {
  int len = mavlink_serial_->write_message(message);
  write_count_++;

  return len;
}

void AutopilotInterface::write_setpoint() {
  // pull from position target
  mavlink_set_position_target_local_ned_t sp = current_setpoint;

  // double check some system parameters
  if (not sp.time_boot_ms) sp.time_boot_ms = (uint32_t)(get_time_usec() / 1000);
  sp.target_system = system_id;
  sp.target_component = autopilot_id;

  mavlink_message_t message;
  mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id,
                                                   &message, &sp);

  if (write_message(message) <= 0) {
    fprintf(stderr, "WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
  }

  return;
}

void AutopilotInterface::enable_offboard_control() {
  // Should only send this command once
  if (control_status_ == false) {
    printf("ENABLE OFFBOARD MODE\n");

    // Sends the command to go off-board
    int success = toggle_offboard_control(true);

    // Check the command was written
    if (success) {
      control_status_ = true;
    } else {
      fprintf(stderr,
              "Error: off-board mode not set, could not write message\n");
    }

    printf("\n");
  }
}

void AutopilotInterface::disable_offboard_control() {
  // Should only send this command once
  if (control_status_ == true) {
    printf("DISABLE OFFBOARD MODE\n");

    // Sends the command to stop off-board
    int success = toggle_offboard_control(false);

    // Check the command was written
    if (success)
      control_status_ = false;
    else {
      fprintf(stderr,
              "Error: off-board mode not set, could not write message\n");
    }

    printf("\n");
  }
}

int AutopilotInterface::toggle_offboard_control(bool flag) {
  // Prepare command for off-board mode
  mavlink_command_long_t com = {0};
  com.target_system = system_id;
  com.target_component = autopilot_id;
  com.command = MAV_CMD_NAV_GUIDED_ENABLE;
  com.confirmation = true;
  com.param1 = (float)flag;

  // Encode
  mavlink_message_t message;
  mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

  // Send the message
  int len = mavlink_serial_->write_message(message);

  return len;
}

void AutopilotInterface::start() {
  int result;

  mavlink_serial_->start();

  if (mavlink_serial_->status != 1) {
    fprintf(stderr, "ERROR: serial port not open\n");
    throw 1;
  }

  printf("START READ THREAD \n");

  result = pthread_create(&read_tid_, NULL,
                          &start_autopilot_interface_read_thread, this);
  if (result) throw result;

  // now we're reading messages
  printf("\n");

  printf("CHECK FOR MESSAGES\n");

  while (not current_messages.sysid) {
    if (time_to_exit_) return;
    usleep(1e6 / 2);
  }

  printf("Found\n");

  // now we know autopilot is sending messages
  printf("\n");

  // This comes from the heartbeat, which in theory should only come from
  // the autopilot we're directly connected to it.  If there is more than one
  // vehicle then we can't expect to discover id's like this.
  // In which case set the id's manually.

  // System ID
  if (not system_id) {
    system_id = current_messages.sysid;
    printf("GOT VEHICLE SYSTEM ID: %i\n", system_id);
  }

  // Component ID
  if (not autopilot_id) {
    autopilot_id = current_messages.compid;
    printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
    printf("\n");
  }

  // Wait for initial position ned
  while (not(current_messages.time_stamps.local_position_ned &&
             current_messages.time_stamps.attitude)) {
    if (time_to_exit_) return;
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

  printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x,
         initial_position.y, initial_position.z);
  printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
  printf("\n");

  printf("START WRITE THREAD \n");

  result = pthread_create(&write_tid_, NULL,
                          &start_autopilot_interface_write_thread, this);
  if (result) throw result;

  // Wait for write thread to be started.
  while (not writing_status_) usleep(1e6 / 10);

  printf("\n");

  return;
}

void AutopilotInterface::stop() {
  printf("CLOSE THREADS\n");

  time_to_exit_ = true;

  pthread_join(read_tid_, NULL);
  pthread_join(write_tid_, NULL);

  printf("\n");

  mavlink_serial_->stop();
}

void AutopilotInterface::start_read_thread() {
  if (reading_status_ != 0) {
    fprintf(stderr, "read thread already running\n");
    return;
  } else {
    read_thread();
    return;
  }
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
  disable_offboard_control();

  try {
    stop();
  } catch (int error) {
    fprintf(stderr, "Warning, could not stop autopilot interface\n");
  }
}

void AutopilotInterface::read_thread() {
  reading_status_ = true;

  while (!time_to_exit_) {
    read_messages();
    usleep(1e6 / 100);
  }

  reading_status_ = false;

  return;
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

  while (!time_to_exit_) {
    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe.

    usleep(1e6 / 10);
    write_setpoint();
  }

  writing_status_ = false;

  return;
}

void *start_autopilot_interface_read_thread(void *args) {
  AutopilotInterface *autopilot_interface = (AutopilotInterface *)args;
  autopilot_interface->start_read_thread();

  return NULL;
}

void *start_autopilot_interface_write_thread(void *args) {
  AutopilotInterface *autopilot_interface = (AutopilotInterface *)args;
  autopilot_interface->start_write_thread();

  return NULL;
}

}  // namespace autopilot_interface
}  // namespace io
}  // namespace control
}  // namespace spinny
