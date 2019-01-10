#include "ground_communicator.h"

namespace src {
namespace controls {
namespace ground_communicator {

GroundCommunicator *socketio_ground_communicator;

void on_connect() { socketio_ground_communicator->OnConnect(); }

void on_fail() { socketio_ground_communicator->OnFail(); }

void connect() { socketio_ground_communicator->ConnectToGround(); }

GroundCommunicator::GroundCommunicator() :
    phased_loop_(1e2),
    running_(false),
    last_serial_telemetry_sent_(0),
    sensors_receiver_("ipc:///tmp/uasatucla_sensors.ipc", 5),
    goal_receiver_("ipc:///tmp/uasatucla_goal.ipc", 5),
    output_receiver_("ipc:///tmp/uasatucla_output.ipc", 5),
    goal_sender_("ipc:///tmp/uasatucla_goal.ipc") {

  socketio_ground_communicator = this;

  client_.set_open_listener(on_connect);
  LOG_LINE("ground_communicator started.");
  SetGoal(INIT);

  ::std::thread ground_socket_thread(connect);
  ground_socket_thread.join();
}

void GroundCommunicator::ConnectToGround() {
#ifdef UAS_AT_UCLA_DEPLOYMENT
  client_.connect("http://192.168.2.20:8081");
#else
  // client_.connect("http://0.0.0.0:8081");
  client_.connect("http://192.168.2.20:8081");
#endif
}

void GroundCommunicator::Run() {
  running_ = true;

  sensors_receiver_.Connect();
  goal_receiver_.Connect();
  output_receiver_.Connect();
  goal_sender_.Connect();

  while (running_) {
    RunIteration();

    phased_loop_.SleepUntilNext();
  }
}

void GroundCommunicator::RunIteration() {
  // Send out latest goal.
  goal_sender_.Send(goal_);

  // Send telemetry to ground.
  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  ::sio::message::ptr all_data = ::sio::object_message::create();
  ::sio::message::ptr telemetry = ::sio::object_message::create();

  // Track which message queues are actually sending things.
  bool send_sensors = false;
  bool send_goal = false;
  bool send_output = false;

  // Fetch the latest data from queues with messages.
  if (sensors_receiver_.HasMessages()) {
    ::src::controls::UasMessage message = sensors_receiver_.GetLatest();
    ::src::controls::Sensors sensors = message.sensors();

    if (sensors.has_latitude()) { // sometimes all the fields are missing???
      send_sensors = true;

      ::std::string sensors_serialized;
      sensors.SerializeToString(&sensors_serialized);

      telemetry->get_map()["sensors"] = ::sio::string_message::create(
          ::lib::base64_tools::Encode(sensors_serialized));

      if (current_time - last_serial_telemetry_sent_ > 0.3) {
        // Time to send another serial telemetry message.
        ::lib::serial_comms::SerialCommsMessage message;
        message.set_latitude(sensors.latitude());
        message.set_longitude(sensors.longitude());
        message.set_altitude(sensors.relative_altitude());
        message.set_heading(sensors.heading());

        ::std::cout << "Lat: " << message.latitude()
                    << " Lng: " << message.longitude()
                    << " Alt: " << message.altitude()
                    << " Heading: " << message.heading() << ::std::endl;

        serial_comms_bridge_.SendData(message);

        last_serial_telemetry_sent_ = current_time;
      }
    }
  }

  if (goal_receiver_.HasMessages()) {
    send_goal = true;

    ::src::controls::Goal goal = goal_receiver_.GetLatest();
    ::std::string goal_serialized;
    goal.SerializeToString(&goal_serialized);

    telemetry->get_map()["goal"] = ::sio::string_message::create(
        ::lib::base64_tools::Encode(goal_serialized));
  }

  if (output_receiver_.HasMessages()) {
    send_output = true;

    ::src::controls::Output output = output_receiver_.GetLatest();
    ::std::string output_serialized;
    output.SerializeToString(&output_serialized);

    telemetry->get_map()["output"] = ::sio::string_message::create(
        ::lib::base64_tools::Encode(output_serialized));
  }

  LOG_LINE("sending telemetry: "               //
           << (send_sensors ? "sensors " : "") //
           << (send_goal ? "goal " : "")       //
           << (send_output ? "output " : ""));

  // Grab information about the latest mission.
  ::lib::mission_manager::Mission mission =
      mission_message_queue_sender_.GetMission();
  ::std::string serialized_mission;
  mission.SerializeToString(&serialized_mission);

  const unsigned char *serialized_mission_cstr =
      reinterpret_cast<const unsigned char *>(serialized_mission.c_str());
  ::std::string mission_base64 = ::lib::base64_tools::Encode(
      serialized_mission_cstr, serialized_mission.length());

  // Package telemetry data for sending to ground system.
  all_data->get_map()["telemetry"] = telemetry;
  all_data->get_map()["mission"] = sio::string_message::create(mission_base64);

  client_.socket("drone")->emit("telemetry", all_data);
}

void GroundCommunicator::OnConnect() {
  LOG_LINE("Someone connected to ground_communicator");

  client_.socket("drone")->on(
      "drone_execute_commands",
      ::sio::socket::event_listener_aux(
          [&](::std::string const &name, ::sio::message::ptr const &data,
              bool isAck, ::sio::message::list &ack_resp) {
            (void)name;
            (void)isAck;
            (void)ack_resp;

            ::lib::mission_manager::GroundData ground_data;
            ::lib::mission_manager::Mission *mission =
                new ::lib::mission_manager::Mission();

            ::std::string serialized_protobuf_mission = data->get_string();

            serialized_protobuf_mission =
                ::lib::base64_tools::Decode(serialized_protobuf_mission);

            mission->ParseFromString(serialized_protobuf_mission);
            ground_data.set_allocated_mission(mission);

            mission_message_queue_sender_.SendData(ground_data);

            SetState("MISSION");
          }));

  client_.socket("drone")->on(
      "interop_data",
      ::sio::socket::event_listener_aux(
          [&](::std::string const &name, ::sio::message::ptr const &data,
              bool isAck, ::sio::message::list &ack_resp) {
            (void)name;
            (void)isAck;
            (void)ack_resp;

            ::lib::mission_manager::GroundData ground_data;
            ::lib::mission_manager::Obstacles *obstacles =
                new ::lib::mission_manager::Obstacles();

            ::std::string serialized_protobuf_obstacles = data->get_string();

            serialized_protobuf_obstacles =
                ::lib::base64_tools::Decode(serialized_protobuf_obstacles);

            obstacles->ParseFromString(serialized_protobuf_obstacles);
            ground_data.set_allocated_obstacles(obstacles);
            mission_message_queue_sender_.SendData(ground_data);
          }));

  client_.socket("drone")->on(
      "drone_set_state",
      ::sio::socket::event_listener_aux(
          [&](::std::string const &name, ::sio::message::ptr const &data,
              bool isAck, ::sio::message::list &ack_resp) {
            (void)name;
            (void)isAck;
            (void)ack_resp;

            SetState(data->get_map()["state"]->get_string());
          }));
}

void GroundCommunicator::OnFail() { ::std::cout << "socketio failed! :(\n"; }

void GroundCommunicator::SetGoal(GoalState new_state) {
  double time = ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
                    ::std::chrono::system_clock::now().time_since_epoch())
                    .count() /
                1e9;

  goal_.set_run_mission(false);

  switch (new_state) {
    case INIT:
      goal_.set_run_mission(false);
      goal_.set_trigger_failsafe(false);
      goal_.set_trigger_throttle_cut(false);
      goal_.set_trigger_takeoff(0);
      goal_.set_trigger_hold(0);
      goal_.set_trigger_offboard(0);
      goal_.set_trigger_rtl(0);
      goal_.set_trigger_land(0);
      goal_.set_trigger_arm(0);
      goal_.set_trigger_disarm(0);
      goal_.set_trigger_alarm(0);
      goal_.set_trigger_bomb_drop(0);
      goal_.set_trigger_dslr(0);
      SetGoal(STANDBY);
      break;

    case STANDBY:
      goal_.set_run_mission(false);
      break;

    case RUN_MISSION:
      goal_.set_run_mission(true);
      break;

    case LAND:
      goal_.set_run_mission(false);
      goal_.set_trigger_land(time);
      LOG_LINE("GOT LAND @ TIME " << time);
      break;

    case FAILSAFE:
      goal_.set_run_mission(false);
      goal_.set_trigger_failsafe(true);
      goal_.set_trigger_throttle_cut(false);
      break;

    case THROTTLE_CUT:
      goal_.set_run_mission(false);
      goal_.set_trigger_failsafe(false);
      goal_.set_trigger_throttle_cut(true);
      break;

    case TAKEOFF:
      goal_.set_trigger_takeoff(time);
      LOG_LINE("GOT TAKEOFF @ TIME " << time);
      break;

    case HOLD:
      goal_.set_trigger_hold(time);
      LOG_LINE("GOT HOLD @ TIME " << time);
      break;

    case OFFBOARD:
      goal_.set_trigger_offboard(time);
      LOG_LINE("GOT OFFBOARD @ TIME " << time);
      break;

    case RTL:
      goal_.set_trigger_rtl(time);
      LOG_LINE("GOT RTL @ TIME " << time);
      break;

    case ARM:
      goal_.set_trigger_arm(time);
      LOG_LINE("GOT ARM @ TIME " << time);
      break;

    case DISARM:
      goal_.set_trigger_disarm(time);
      LOG_LINE("GOT DISARM @ TIME " << time);
      break;

    case ALARM:
      goal_.set_trigger_alarm(time);
      LOG_LINE("GOT ALARM @ TIME " << time);
      break;

    case BOMB_DROP:
      goal_.set_trigger_bomb_drop(time);
      LOG_LINE("GOT BOMB DROP @ TIME " << time);
      break;

    case DSLR:
      goal_.set_trigger_dslr(time);
      LOG_LINE("GOT DSLR TRIGGER @ TIME " << time);
      break;
  }
}

void GroundCommunicator::SetState(::std::string new_state_string) {
  GoalState new_state;

  if (new_state_string == "MISSION") {
    // TODO(comran): Check if at safe altitude.
    new_state = RUN_MISSION;
  } else if (new_state_string == "FAILSAFE") {
    new_state = FAILSAFE;
  } else if (new_state_string == "THROTTLE CUT") {
    new_state = THROTTLE_CUT;
  } else if (new_state_string == "TAKEOFF") {
    new_state = TAKEOFF;
  } else if (new_state_string == "HOLD") {
    new_state = HOLD;
  } else if (new_state_string == "OFFBOARD") {
    new_state = OFFBOARD;
  } else if (new_state_string == "RTL") {
    new_state = RTL;
  } else if (new_state_string == "LAND") {
    new_state = LAND;
  } else if (new_state_string == "ARM") {
    new_state = ARM;
  } else if (new_state_string == "DISARM") {
    new_state = DISARM;
  } else if (new_state_string == "ALARM") {
    new_state = ALARM;
  } else if (new_state_string == "BOMB_DROP") {
    new_state = BOMB_DROP;
  } else if (new_state_string == "DSLR") {
    new_state = DSLR;
  } else {
    ::std::cerr << "Unknown state: " << new_state_string << ::std::endl;
    return;
  }

  if ((state_ == FAILSAFE && new_state != THROTTLE_CUT) ||
      (state_ == THROTTLE_CUT)) {
    ::std::cerr << "Could not override higher order emergency state."
                << ::std::endl;

    return;
  }

  state_ = new_state;
  SetGoal(new_state);
}

} // namespace ground_communicator
} // namespace controls
} // namespace src
