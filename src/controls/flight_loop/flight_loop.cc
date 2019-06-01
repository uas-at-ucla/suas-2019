#include "flight_loop.h"

namespace src {
namespace controls {
namespace flight_loop {

FlightLoop::FlightLoop() :
    running_(false),
    last_loop_(0),
    did_alarm_(false),
    did_arm_(false),
    last_bomb_drop_(0),
    last_dslr_(0),
    sensors_subscriber_(ros_node_handle_.subscribe(
        io::kRosSensorsTopic, io::kRosMessageQueueSize,
        &FlightLoop::RunIteration, this)),
    output_publisher_(ros_node_handle_.advertise<::src::controls::Output>(
        io::kRosOutputTopic, io::kRosMessageQueueSize)) {

  ROS_INFO("Flight loop initialized!");
}

void FlightLoop::RunIteration(::src::controls::Sensors sensors) {
  ::src::controls::Output output = GenerateDefaultOutput();

  // state_machine_.Handle(sensors, goal, output);
  // WriteActuators(sensors, goal, output);

  LogProtobufMessage("SENSORS", sensors);
  LogProtobufMessage("OUTPUT", output);

  output_publisher_.publish(output);
}

void FlightLoop::MonitorLoopFrequency(::src::controls::Sensors sensors) {
  // LOG_LINE("Flight Loop dt: " << ::std::setprecision(14)
  //                            << sensors.time() - last_loop_ - 0.01);

  if (sensors.time() - last_loop_ > 0.01 + 0.002) {
    //  LOG_LINE("Flight LOOP RUNNING SLOW: dt: "
    //           << std::setprecision(14) << sensors.time() - last_loop_ -
    //           0.01);
  }
  last_loop_ = sensors.time();
}

::src::controls::Output FlightLoop::GenerateDefaultOutput() {
  ::src::controls::Output output;

  // Set state to integer representation of the current state of the flight
  // loop.
  output.set_state(0);
  output.set_flight_time(0);
  output.set_current_command_index(0);

  output.set_send_offboard(false);
  output.set_velocity_x(0);
  output.set_velocity_y(0);
  output.set_velocity_z(0);
  output.set_yaw_setpoint(0);

  output.set_gimbal_angle(kDefaultGimbalAngle);
  output.set_bomb_drop(false);
  output.set_alarm(false);
  output.set_dslr(false);

  output.set_trigger_takeoff(0);
  output.set_trigger_hold(0);
  output.set_trigger_offboard(0);
  output.set_trigger_rtl(0);
  output.set_trigger_land(0);
  output.set_trigger_arm(0);
  output.set_trigger_disarm(0);

  return output;
}

void FlightLoop::WriteActuators(::src::controls::Sensors &sensors,
                                ::src::controls::Goal &goal,
                                ::src::controls::Output &output) {
  (void)sensors;
  (void)goal;
  (void)output;
  // // Handle alarm.
  // output.set_alarm(alarm_.ShouldAlarm());

  // if (goal.trigger_alarm() + 0.05 > sensors.time()) {
  //   if (!did_alarm_) {
  //     did_alarm_ = true;
  //     alarm_.AddAlert({0.30, 0.30});
  //     //    LOG_LINE("Alarm was manually triggered");
  //   }
  // } else {
  //   did_alarm_ = false;
  // }

  // if (sensors.armed() && !did_arm_) {
  //   // Send out a chirp if the Pixhawk just got armed.
  //   did_arm_ = true;
  //   alarm_.AddAlert({0.03, 0.25});
  // }

  // if (!sensors.armed()) {
  //   did_arm_ = false;
  // }

  // // Handle bomb drop.
  // last_bomb_drop_ = ::std::max(last_bomb_drop_, goal.trigger_bomb_drop());

  // output.set_bomb_drop(false);
  // if (last_bomb_drop_ <= sensors.time() &&
  //     last_bomb_drop_ + 5.0 > sensors.time()) {
  //   output.set_bomb_drop(true);
  // }

  // // Handle dslr.
  // output.set_dslr(false);
  // last_dslr_ = ::std::max(last_dslr_, goal.trigger_dslr());
  // if (last_dslr_ <= sensors.time() && last_dslr_ + 15.0 > sensors.time()) {
  //   output.set_dslr(true);
  // }
}

void FlightLoop::LogProtobufMessage(::std::string name,
                                    ::google::protobuf::Message &message) {
  ::std::ostringstream output;

  ::std::string sensors_string;
  ::google::protobuf::TextFormat::PrintToString(message, &sensors_string);

  ::std::vector<::std::string> sensors_split;
  ::boost::split(sensors_split, sensors_string,
                 [](char c) { return c == '\n'; });
  sensors_split.insert(sensors_split.begin(), "");
  sensors_split.pop_back();

  for (::std::string field : sensors_split) {
    output << name << "... " << field << "\n";
  }

  ROS_DEBUG_STREAM(output.str());
}

} // namespace flight_loop
} // namespace controls
} // namespace src
