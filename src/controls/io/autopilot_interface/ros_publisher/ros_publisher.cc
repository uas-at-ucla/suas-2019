#include "ros_publisher.h"

namespace src {
namespace controls {
namespace io {
namespace autopilot_interface {
namespace ros_publisher {

RosPublisher::RosPublisher() :
    heartbeat_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsHeartbeat>(
            "/sensors/local_position_ned", 1000)),
    sys_status_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsSysStatus>(
            "/sensors/local_position_ned", 1000)),
    battery_status_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsBatteryStatus>(
            "/sensors/battery_status", 1000)),
    radio_status_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsRadioStatus>(
            "/sensors/radio_status", 1000)),
    local_position_ned_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsLocalPositionNed>(
            "/sensors/local_position_ned", 1000)),
    global_position_int_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsGlobalPositionInt>(
            "/sensors/global_position_int", 1000)),
    gps_raw_int_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsGpsRawInt>(
            "/sensors/gps_raw_int", 1000)),
    highres_imu_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsHighresImu>(
            "/sensors/highres_imu", 1000)),
    attitude_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsAttitude>(
            "/sensors/attitude", 1000)),
    vfr_hud_publisher_(
        ros_node_handle_.advertise<::src::controls::SensorsVfrHud>(
            "/sensors/vfr_hud", 1000)),
    actuator_control_target_publisher_(
        ros_node_handle_
            .advertise<::src::controls::SensorsActuatorControlTarget>(
                "/sensors/actuator_control_target", 1000)) {}

void RosPublisher::WriteMessage(const mavlink_message_t *msg,
                                const ::mavconn::Framing framing) {
  (void)framing;

  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(msg, &heartbeat);

      ::src::controls::SensorsHeartbeat heartbeat_proto;
      heartbeat_proto.set_autopilot(heartbeat.autopilot);
      heartbeat_proto.set_base_mode(heartbeat.base_mode);
      heartbeat_proto.set_custom_mode(heartbeat.custom_mode);
      heartbeat_proto.set_mavlink_version(heartbeat.mavlink_version);
      heartbeat_proto.set_system_status(heartbeat.system_status);
      heartbeat_proto.set_type(heartbeat.type);

      heartbeat_publisher_.publish(heartbeat_proto);

      break;
    }

    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t sys_status;
      mavlink_msg_sys_status_decode(msg, &sys_status);

      ::src::controls::SensorsSysStatus sys_status_proto;
      sys_status_proto.set_battery_remaining(sys_status.battery_remaining);
      sys_status_proto.set_drop_rate_comm(sys_status.drop_rate_comm);
      sys_status_proto.set_errors_comm(sys_status.errors_comm);
      sys_status_proto.set_errors_count1(sys_status.errors_count1);
      sys_status_proto.set_errors_count2(sys_status.errors_count2);
      sys_status_proto.set_errors_count3(sys_status.errors_count3);
      sys_status_proto.set_errors_count4(sys_status.errors_count4);
      sys_status_proto.set_load(sys_status.load);
      sys_status_proto.set_onboard_control_sensors_enabled(
          sys_status.onboard_control_sensors_enabled);
      sys_status_proto.set_onboard_control_sensors_health(
          sys_status.onboard_control_sensors_health);
      sys_status_proto.set_onboard_control_sensors_present(
          sys_status.onboard_control_sensors_present);
      sys_status_proto.set_voltage_battery(sys_status.voltage_battery);

      sys_status_publisher_.publish(sys_status_proto);

      break;
    }

    case MAVLINK_MSG_ID_BATTERY_STATUS: {
      mavlink_battery_status_t battery_status;
      mavlink_msg_battery_status_decode(msg, &battery_status);

      ::src::controls::SensorsBatteryStatus battery_status_proto;

      battery_status_proto.set_battery_function(
          battery_status.battery_function);
      battery_status_proto.set_battery_remaining(
          battery_status.battery_remaining);
      battery_status_proto.set_charge_state(battery_status.charge_state);
      battery_status_proto.set_current_battery(battery_status.current_battery);
      battery_status_proto.set_current_consumed(
          battery_status.current_consumed);
      battery_status_proto.set_energy_consumed(battery_status.energy_consumed);
      battery_status_proto.set_id(battery_status.id);
      battery_status_proto.set_temperature(battery_status.temperature);
      battery_status_proto.set_time_remaining(battery_status.time_remaining);
      battery_status_proto.set_type(battery_status.type);
      for (size_t i = 0; i < sizeof(battery_status.voltages) /
                                 sizeof(*battery_status.voltages);
           i++) {
        battery_status_proto.add_voltages(battery_status.voltages[i]);
      }

      battery_status_publisher_.publish(battery_status_proto);

      break;
    }

    case MAVLINK_MSG_ID_RADIO_STATUS: {
      mavlink_radio_status_t radio_status;
      mavlink_msg_radio_status_decode(msg, &radio_status);

      ::src::controls::SensorsRadioStatus radio_status_proto;

      radio_status_proto.set_fixed(radio_status.fixed);
      radio_status_proto.set_noise(radio_status.noise);
      radio_status_proto.set_remnoise(radio_status.remnoise);
      radio_status_proto.set_remrssi(radio_status.remrssi);
      radio_status_proto.set_rssi(radio_status.rssi);
      radio_status_proto.set_rxerrors(radio_status.rxerrors);
      radio_status_proto.set_txbuf(radio_status.txbuf);

      radio_status_publisher_.publish(radio_status_proto);

      break;
    }

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
      mavlink_local_position_ned_t local_position_ned;
      mavlink_msg_local_position_ned_decode(msg, &local_position_ned);

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

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t global_position_ned;
      mavlink_msg_global_position_int_decode(msg, &global_position_ned);

      ::src::controls::SensorsGlobalPositionInt global_position_int_proto;
      global_position_int_proto.set_alt(global_position_ned.alt);
      global_position_int_proto.set_hdg(global_position_ned.hdg);
      global_position_int_proto.set_lat(global_position_ned.lat);
      global_position_int_proto.set_lon(global_position_ned.lon);
      global_position_int_proto.set_relative_alt(
          global_position_ned.relative_alt);
      global_position_int_proto.set_time_boot_ms(
          global_position_ned.time_boot_ms);
      global_position_int_proto.set_vx(global_position_ned.vx);
      global_position_int_proto.set_vy(global_position_ned.vy);
      global_position_int_proto.set_vz(global_position_ned.vz);

      global_position_int_publisher_.publish(global_position_int_proto);
      break;
    }

    case MAVLINK_MSG_ID_GPS_RAW_INT: {
      mavlink_gps_raw_int_t gps_raw_int;
      mavlink_msg_gps_raw_int_decode(msg, &gps_raw_int);

      ::src::controls::SensorsGpsRawInt gps_raw_int_proto;
      gps_raw_int_proto.set_alt(gps_raw_int.alt);
      gps_raw_int_proto.set_alt_ellipsoid(gps_raw_int.alt_ellipsoid);
      gps_raw_int_proto.set_cog(gps_raw_int.cog);
      gps_raw_int_proto.set_eph(gps_raw_int.eph);
      gps_raw_int_proto.set_epv(gps_raw_int.epv);
      gps_raw_int_proto.set_fix_type(gps_raw_int.fix_type);
      gps_raw_int_proto.set_h_acc(gps_raw_int.h_acc);
      gps_raw_int_proto.set_hdg_acc(gps_raw_int.hdg_acc);
      gps_raw_int_proto.set_lat(gps_raw_int.lat);
      gps_raw_int_proto.set_lon(gps_raw_int.lon);
      gps_raw_int_proto.set_satellites_visible(gps_raw_int.satellites_visible);
      gps_raw_int_proto.set_time_usec(gps_raw_int.time_usec);
      gps_raw_int_proto.set_v_acc(gps_raw_int.v_acc);
      gps_raw_int_proto.set_vel(gps_raw_int.vel);
      gps_raw_int_proto.set_vel_acc(gps_raw_int.vel_acc);

      gps_raw_int_publisher_.publish(gps_raw_int_proto);
      break;
    }

    case MAVLINK_MSG_ID_HIGHRES_IMU: {
      mavlink_highres_imu_t highres_imu;
      mavlink_msg_highres_imu_decode(msg, &highres_imu);

      ::src::controls::SensorsHighresImu highres_imu_proto;

      highres_imu_proto.set_abs_pressure(highres_imu.abs_pressure);
      highres_imu_proto.set_diff_pressure(highres_imu.diff_pressure);
      highres_imu_proto.set_fields_updated(highres_imu.fields_updated);
      highres_imu_proto.set_pressure_alt(highres_imu.pressure_alt);
      highres_imu_proto.set_temperature(highres_imu.temperature);
      highres_imu_proto.set_time_usec(highres_imu.time_usec);
      highres_imu_proto.set_xgyro(highres_imu.xgyro);
      highres_imu_proto.set_zgyro(highres_imu.zgyro);
      highres_imu_proto.set_ygyro(highres_imu.ygyro);
      highres_imu_proto.set_xacc(highres_imu.xacc);
      highres_imu_proto.set_yacc(highres_imu.yacc);
      highres_imu_proto.set_zacc(highres_imu.zacc);
      highres_imu_proto.set_xmag(highres_imu.xmag);
      highres_imu_proto.set_ymag(highres_imu.ymag);
      highres_imu_proto.set_zmag(highres_imu.zmag);

      highres_imu_publisher_.publish(highres_imu_proto);
      break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t attitude;
      mavlink_msg_attitude_decode(msg, &attitude);

      ::src::controls::SensorsAttitude attitude_proto;

      attitude_proto.set_pitch(attitude.pitch);
      attitude_proto.set_roll(attitude.roll);
      attitude_proto.set_yaw(attitude.yaw);
      attitude_proto.set_pitchspeed(attitude.pitchspeed);
      attitude_proto.set_rollspeed(attitude.rollspeed);
      attitude_proto.set_yawspeed(attitude.yawspeed);

      attitude_publisher_.publish(attitude_proto);
      break;
    }

    case MAVLINK_MSG_ID_VFR_HUD: {
      mavlink_vfr_hud_t vfr_hud;
      mavlink_msg_vfr_hud_decode(msg, &vfr_hud);

      ::src::controls::SensorsVfrHud vfr_hud_proto;
      vfr_hud_proto.set_airspeed(vfr_hud.airspeed);
      vfr_hud_proto.set_alt(vfr_hud.alt);
      vfr_hud_proto.set_climb(vfr_hud.climb);
      vfr_hud_proto.set_groundspeed(vfr_hud.groundspeed);
      vfr_hud_proto.set_heading(vfr_hud.heading);
      vfr_hud_proto.set_throttle(vfr_hud.throttle);

      vfr_hud_publisher_.publish(vfr_hud_proto);
      break;
    }

    case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET: {
      mavlink_actuator_control_target_t actuator_control_target;
      mavlink_msg_actuator_control_target_decode(msg, &actuator_control_target);

      ::src::controls::SensorsActuatorControlTarget
          actuator_control_target_proto;
      for (size_t i = 0; i < sizeof(actuator_control_target.controls) /
                                 sizeof(*actuator_control_target.controls);
           i++) {
        actuator_control_target_proto.add_controls(
            actuator_control_target.controls[i]);
      }

      actuator_control_target_proto.set_group_mlx(
          actuator_control_target.group_mlx);
      actuator_control_target_proto.set_time_usec(
          actuator_control_target.time_usec);

      actuator_control_target_publisher_.publish(actuator_control_target_proto);
      break;
    }

    default:
      break;
  }
}

} // namespace ros_publisher
} // namespace autopilot_interface
} // namespace io
} // namespace controls
} // namespace src
