#include "ros_publisher.h"

namespace src {
namespace controls {
namespace io {
namespace autopilot_interface {
namespace ros_publisher {

RosPublisher::RosPublisher() :
    local_position_ned_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsLocalPositionNed>(
            "/sensors/local_position_ned", 1000)) {}

void RosPublisher::WriteMessage(const mavlink_message_t *msg,
                                         const ::mavconn::Framing framing) {
  (void) framing;
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      // mavlink_msg_heartbeat_decode(msg, &(current_messages.heartbeat));
      // current_messages.time_stamps.heartbeat = get_time_usec();
      break;

    case MAVLINK_MSG_ID_SYS_STATUS:
      // mavlink_msg_sys_status_decode(msg, &(current_messages.sys_status));
      // current_messages.time_stamps.sys_status = get_time_usec();
      break;

    case MAVLINK_MSG_ID_BATTERY_STATUS:
      // mavlink_msg_battery_status_decode(msg,
      //                                   &(current_messages.battery_status));
      // current_messages.time_stamps.battery_status = get_time_usec();
      break;

    case MAVLINK_MSG_ID_RADIO_STATUS:
      // mavlink_msg_radio_status_decode(msg, &(current_messages.radio_status));
      // current_messages.time_stamps.radio_status = get_time_usec();
      break;

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
      mavlink_local_position_ned_t local_position_ned;
      mavlink_msg_local_position_ned_decode(msg, &(local_position_ned));

      ::src::controls::SensorsLocalPositionNed local_position_ned_proto;
      local_position_ned_proto.set_x(local_position_ned.x);
      local_position_ned_proto.set_y(local_position_ned.y);
      local_position_ned_proto.set_z(local_position_ned.z);
      local_position_ned_proto.set_vx(local_position_ned.vx);
      local_position_ned_proto.set_vy(local_position_ned.vy);
      local_position_ned_proto.set_vz(local_position_ned.vz);

      local_position_ned_publisher_.publish(local_position_ned_proto);
      break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      // mavlink_msg_global_position_int_decode(
      //     msg, &(current_messages.global_position_int));
      // current_messages.time_stamps.global_position_int = get_time_usec();
      break;

    case MAVLINK_MSG_ID_GPS_RAW_INT:
      // mavlink_msg_gps_raw_int_decode(msg, &(current_messages.gps_raw_int));
      // current_messages.time_stamps.gps_raw_int = get_time_usec();
      break;

    case MAVLINK_MSG_ID_HIGHRES_IMU:
      // mavlink_msg_highres_imu_decode(msg, &(current_messages.highres_imu));
      // current_messages.time_stamps.highres_imu = get_time_usec();
      break;

    case MAVLINK_MSG_ID_ATTITUDE:
      // mavlink_msg_attitude_decode(msg, &(current_messages.attitude));
      // current_messages.time_stamps.attitude = get_time_usec();
      break;

    case MAVLINK_MSG_ID_VFR_HUD:
      // mavlink_msg_vfr_hud_decode(msg, &(current_messages.vfr_hud));
      // current_messages.time_stamps.vfr_hud = get_time_usec();
      break;

    case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
      // mavlink_msg_actuator_control_target_decode(
      //     msg, &(current_messages.control_target));

      break;

    default:
      break;
  }
}

} // namespace ros_publisher
} // namespace autopilot_interface
} // namespace io
} // namespace controls
} // namespace src
