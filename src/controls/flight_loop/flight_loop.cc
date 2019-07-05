#include "flight_loop.h"

namespace src {
namespace controls {
namespace flight_loop {

FlightLoop::FlightLoop() :
    running_(false),
    last_loop_(0),
    last_proto_log_(0),
    did_alarm_(false),
    did_arm_(false),
    last_bomb_drop_(0),
    last_dslr_(0),
    ros_node_handle_(),
    sensors_subscriber_(
        ros_node_handle_.subscribe(kRosSensorsTopic, kRosMessageQueueSize,
                                   &FlightLoop::RunIteration, this)),
    drone_program_subscriber_(
        ros_node_handle_.subscribe(kRosDroneProgramTopic, kRosMessageQueueSize,
                                   &FlightLoop::DroneProgramReceived, this)),
    output_publisher_(ros_node_handle_.advertise<::src::controls::Output>(
        kRosOutputTopic, kRosMessageQueueSize)) {

  ROS_INFO("Flight loop initialized!");

  // Set initial goal.
  goal_.set_run_mission(false);
}

void FlightLoop::RunIteration(::src::controls::Sensors sensors) {
  // Create a clone of the current goal.
  ::src::controls::Goal goal = GetGoal();
  goal.set_run_mission(sensors.run_uas_mission());

  // Create a default output to return.
  ::src::controls::Output output = GenerateDefaultOutput();

  // Handle the current state.
  state_machine_.Handle(sensors, goal, output);

  // Log protobufs.
  if (sensors.time() > last_proto_log_ + 1.0 / kProtobufLogHz) {
    // LogProtobufMessage("Sensors", sensors);
    // LogProtobufMessage("Goal", goal);
    // LogProtobufMessage("Output", output);
    last_proto_log_ = sensors.time();
  }

  // Send output protobuf back to IO to handle.
  output_publisher_.publish(output);
}

void FlightLoop::DroneProgramReceived(
    ::src::controls::ground_controls::timeline::DroneProgram drone_program) {

  ::std::cout << "GOT MISSION!\n";
  LogProtobufMessage("DroneProgram", drone_program);

  state_machine_.LoadMission(drone_program);
}

void FlightLoop::MonitorLoopFrequency(::src::controls::Sensors sensors) {
  double dt = sensors.time() - last_loop_;

  if (dt > 1.0 / kExpectedFlightLoopHz + kFlightLoopTolerancePeriod) {
    ROS_DEBUG("Flight LOOP RUNNING SLOW: dt: %f", dt);
  }

  last_loop_ = sensors.time();
}

::src::controls::Output FlightLoop::GenerateDefaultOutput() {
  ::src::controls::Output output;

  // Set state to integer representation of the current state of the flight
  // loop.
  output.set_state(0);
  output.set_mission_state(0);
  output.set_flight_time(0);
  output.set_current_command_index(0);

  output.set_send_setpoint(false);
  output.set_setpoint_latitude(0);
  output.set_setpoint_longitude(0);
  output.set_setpoint_altitude(0);
  output.set_setpoint_yaw(0);

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

  output.set_deploy(false);

  output.set_mission_commanded_land(false);

  return output;
}

::src::controls::Goal FlightLoop::GetGoal() {
  Goal goal_copy;

  // Grab lock to prevent ROS from modifying the shared Goal object while a
  // copy is performed.
  {
    ::std::lock_guard<::std::mutex> lock(goal_mutex_);
    goal_copy.CopyFrom(goal_);
  }

  return goal_copy;
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
