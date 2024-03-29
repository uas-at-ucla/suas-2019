#pragma once

#include <string>

namespace src {
namespace controls {
namespace {
// WiringPi GPIO identifiers
static const int kAlarmGPIOPin = 0;

// Time between sending arm triggers to the flight controller.
static constexpr double kTriggerPeriod = 3.0;

// Pigpio GPIO identifiers (same as BCM)
static const int kDeploymentEncoderChannelOne = 8;
static const int kDeploymentEncoderChannelTwo = 7;

static const int kDeploymentMotorReverseGPIOPin = 11;
static const int kDeploymentHotwireGPIOPin = 26;
static const int kGimbalGPIOPin = 23;
static const int kDeploymentLatchServoGPIOPin = 24;
static const int kDeploymentMotorGPIOPin = 27;

static const int kPpmMiddleSignal = 1500;
static const int kDeploymentServoClosed = 1600;
static const int kDeploymentServoOpen = 1000;

// Actuator RC channels.
static const int kRcInMinimumThreshold = 900;
static const int kThrottleRcChannel = 1;
static const int kUasMissionRcChannel = 5;
static const int kAlarmOverrideRcChannel = 7;
static const int kDeploymentMotorRcChannel = 8;
static const int kGimbalMotorRcChannel = 9;

static const int kRcLossAlertTimeout = 5;
static const int kRcLossRtlTimeout = 20;
static const int kRcLossTerminateTimeout = 30;

static const int kAlarmOverrideRcSignalThreshold = 1800;

static const int kWriterPhasedLoopFrequency = 250;
static const double kRcInTimeGap = 1.0 / 5;

static const int kSensorsPublisherRate = 50;
static const double kSensorsPublisherPeriod = 1.0 / kSensorsPublisherRate;

static const double kAlarmChirpDuration = 0.005;

// ROS topic parameters.
static const int kRosMessageQueueSize = 1;
static const double kRosReceiveTolerance = 5;

static constexpr double kActuatorLogHz = 2.0;

static const ::std::string kRosGimbalTopic = "/uasatucla/actuators/gimbal";
static const ::std::string kRosDeploymentMotorTopic =
    "/uasatucla/actuators/deployment_motor";
static const ::std::string kRosLatchTopic = "/uasatucla/actuators/latch";
static const ::std::string kRosHotwireTopic = "/uasatucla/actuators/hotwire";
static const ::std::string kRosAlarmTriggerTopic = "/uasatucla/actuators/alarm";
static const ::std::string kRosSensorsTopic = "/uasatucla/proto/sensors";
static const ::std::string kRosOutputTopic = "/uasatucla/proto/output";

static const ::std::string kRosRcInTopic = "/mavros/rc/in";
static const ::std::string kRosBatteryStatusTopic = "/mavros/battery";
static const ::std::string kRosStateTopic = "/mavros/state";
static const ::std::string kRosImuTopic = "/mavros/imu/data";
static const ::std::string kRosGlobalPositionSetpointTopic =
    "/mavros/setpoint_position/global";
static const ::std::string kRosGlobalPositionTopic =
    "/mavros/global_position/global";
static const ::std::string kRosSetModeService = "/mavros/set_mode";
static const ::std::string kRosArmService = "/mavros/cmd/arming";
static const ::std::string kRosCmdIntService = "/mavros/cmd/command_int";
static const ::std::string kRosTakeoffService = "/mavros/cmd/takeoff";
static const ::std::string kRosDiagnosticsTopic = "/diagnostics";
static const ::std::string kRosVfrHudTopic = "/mavros/vfr_hud";
static const ::std::string kRosCompassHeadingTopic =
    "/mavros/global_position/compass_hdg";
static const ::std::string kRosVelocityTopic =
    "/mavros/local_position/velocity_local"; // Note: this is different from the
                                             // mavros docs!
static const ::std::string kRosAltitudeTopic = "/mavros/altitude";
static const ::std::string kRosHomePositionTopic = "/mavros/home_position/home";

static const ::std::string kRosDroneProgramTopic =
    "/uasatucla/proto/drone_program";
static const ::std::string kRosMissionStatusTopic = "/uasatucla/mission_status";
static const ::std::string kRosTakePhotoTopic = "/uasatucla/camera/take_photo";

// Pixhawk interface parameters.
static constexpr double kPixhawkGlobalSetpointMaxHz = 10.0;

// Pixhawk commands and custom modes.
// Documentation: https://dev.px4.io/en/concept/flight_modes.html
// TODO(comran): Use custom mode constants provided internally by Mavros.
static const ::std::string kPixhawkArmCommand = "ARM";
static const ::std::string kPixhawkFlightTermCommand = "FLIGHTTERM";
static const ::std::string kPixhawkCustomModeManual = "MANUAL";
static const ::std::string kPixhawkCustomModeAcro = "ACRO";
static const ::std::string kPixhawkCustomModeAltitudeControl = "ALTCTL";
static const ::std::string kPixhawkCustomModePositionControl = "POSCTL";
static const ::std::string kPixhawkCustomModeOffboard = "OFFBOARD";
static const ::std::string kPixhawkCustomModeStabilized = "STABILIZED";
static const ::std::string kPixhawkCustomModeRattitude = "RATTITUDE";
static const ::std::string kPixhawkCustomModeMission = "AUTO.MISSION";
static const ::std::string kPixhawkCustomModeLoiter = "AUTO.LOITER";
static const ::std::string kPixhawkCustomModeReturnToLand = "AUTO.RTL";
static const ::std::string kPixhawkCustomModeLand = "AUTO.LAND";
static const ::std::string kPixhawkCustomModeReturnToGroundStation =
    "AUTO.RTGS";
static const ::std::string kPixhawkCustomModeAutoReady = "AUTO.READY";
static const ::std::string kPixhawkCustomModeTakeoff = "AUTO.TAKEOFF";
static const ::std::string kPixhawkCustomModeFollowTarget =
    "AUTO.FOLLOW_TARGET";
static const ::std::string kPixhawkCustomModePrecland = "AUTO.PRECLAND";
} // namespace
} // namespace controls
} // namespace src
