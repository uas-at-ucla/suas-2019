#include "ros_subscriber.h"

namespace src {
namespace controls {
namespace io {
namespace autopilot_interface {
namespace ros_subscriber {

RosSubscriber::RosSubscriber() :
    heartbeat_subscriber_(
            "/sensors/local_position_ned", 1000, &RosSubscriber::ReadMessage, this),
    sys_status_subscriber_(
        ros_node_handle_.advertise<::src::controls::SensorsSysStatus>(
            "/sensors/local_position_ned", 1000)),
    battery_status_subscriber_(
        ros_node_handle_.advertise<::src::controls::SensorsBatteryStatus>(
            "/sensors/battery_status", 1000)),
    radio_status_subscriber_(
        ros_node_handle_.advertise<::src::controls::SensorsRadioStatus>(
            "/sensors/radio_status", 1000)),
    local_position_ned_subscriber_(
        ros_node_handle_.advertise<::src::controls::SensorsLocalPositionNed>(
            "/sensors/local_position_ned", 1000)),
    global_position_int_subscriber_(
        ros_node_handle_.advertise<::src::controls::SensorsGlobalPositionInt>(
            "/sensors/global_position_int", 1000)),
    gps_raw_int_subscriber_(
        ros_node_handle_.advertise<::src::controls::SensorsGpsRawInt>(
            "/sensors/gps_raw_int", 1000)),
    highres_imu_subscriber_(
        ros_node_handle_.advertise<::src::controls::SensorsHighresImu>(
            "/sensors/highres_imu", 1000)),
    attitude_subscriber_(
        ros_node_handle_.advertise<::src::controls::SensorsAttitude>(
            "/sensors/attitude", 1000)),
    vfr_hud_subscriber_(
        ros_node_handle_.advertise<::src::controls::SensorsVfrHud>(
            "/sensors/vfr_hud", 1000)),
    actuator_control_target_subscriber_(
        ros_node_handle_
            .advertise<::src::controls::SensorsActuatorControlTarget>(
                "/sensors/actuator_control_target", 1000)) {}

void RosSubscriber::ReadMessage(const ::src::controls::Message &message) {

}

} // namespace ros_subscriber
} // namespace autopilot_interface
} // namespace io
} // namespace controls
} // namespace src
