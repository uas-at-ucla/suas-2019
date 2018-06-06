#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

#include "lib/mavconn_udp/interface.h"

#include <signal.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <common/mavlink.h>

namespace src {
namespace control {
namespace io {
namespace autopilot_interface {

 // Bitmasks to indicate what type of message is being sent to the vehicle.
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION 0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY 0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION \
  0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE 0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE 0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE 0b0000010111111111

uint64_t get_time_usec();
void set_position(float x, float y, float z,
                  mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz,
                  mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);

void *start_autopilot_interface_write_thread(void *args);

struct TimeStamps {
  TimeStamps() { reset_timestamps(); }

  uint64_t heartbeat;
  uint64_t sys_status;
  uint64_t battery_status;
  uint64_t radio_status;
  uint64_t local_position_ned;
  uint64_t global_position_int;
  uint64_t gps_raw_int;
  uint64_t position_target_local_ned;
  uint64_t position_target_global_int;
  uint64_t highres_imu;
  uint64_t attitude;
  uint64_t vfr_hud;

  void reset_timestamps() {
    heartbeat = 0;
    sys_status = 0;
    battery_status = 0;
    radio_status = 0;
    local_position_ned = 0;
    global_position_int = 0;
    gps_raw_int = 0;
    position_target_local_ned = 0;
    position_target_global_int = 0;
    highres_imu = 0;
    attitude = 0;
    vfr_hud = 0;
  }
};

struct Mavlink_Messages {
  int sysid;
  int compid;

  mavlink_heartbeat_t heartbeat;
  mavlink_sys_status_t sys_status;
  mavlink_battery_status_t battery_status;
  mavlink_radio_status_t radio_status;
  mavlink_local_position_ned_t local_position_ned;
  mavlink_global_position_int_t global_position_int;
  mavlink_gps_raw_int_t gps_raw_int;
  mavlink_position_target_local_ned_t position_target_local_ned;
  mavlink_position_target_global_int_t position_target_global_int;
  mavlink_highres_imu_t highres_imu;
  mavlink_attitude_t attitude;
  mavlink_vfr_hud_t vfr_hud;
  mavlink_actuator_control_target_t control_target;

  TimeStamps time_stamps;

  void reset_timestamps() { time_stamps.reset_timestamps(); }
};

class AutopilotInterface {
 public:
  AutopilotInterface();
  ~AutopilotInterface();

  int system_id;
  int autopilot_id;
  int companion_id;

  void Arm();
  void Disarm();
  void DoGimbal();
  void Takeoff();
  void Hold();
  void ReturnToLaunch();
  void Offboard();
  void Land();
  void FlightTermination();

  void set_message_period();
  void set_params();
  void set_param(const char id[], float value);

  Mavlink_Messages current_messages;
  mavlink_set_position_target_local_ned_t initial_position;

  void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
  void read_messages();
  void write_message(mavlink_message_t message);

  void start();
  void stop();

  void start_write_thread(void);

  void handle_quit(int sig);

 private:
  ::mavconn::MAVConnInterface::Ptr pixhawk_;

  mavlink_set_position_target_local_ned_t current_setpoint;

  void write_thread(void);

  void write_setpoint();

  pthread_t write_tid_;

  char reading_status_;
  char writing_status_;

  uint64_t write_count_;

  bool time_to_exit_;
};

}  // namespace autopilot_interface
}  // namespace io
}  // namespace control
}  // namespace src

#endif  // AUTOPILOT_INTERFACE_H_
