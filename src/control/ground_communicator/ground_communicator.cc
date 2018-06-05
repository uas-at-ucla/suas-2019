#include <bitset>
#include "ground_communicator.h"

namespace src {
namespace control {
namespace ground_communicator {

// Taken from here:
// https://renenyffenegger.ch/notes/development/Base64/Encoding-and-decoding-base-64-with-cpp
static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

static inline bool is_base64(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

std::string base64_decode(std::string const& encoded_string) {
  int in_len = encoded_string.size();
  int i = 0;
  int j = 0;
  int in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];
  std::string ret;

  while (in_len-- && (encoded_string[in_] != '=') &&
         is_base64(encoded_string[in_])) {
    char_array_4[i++] = encoded_string[in_];
    in_++;
    if (i == 4) {
      for (i = 0; i < 4; i++)
        char_array_4[i] = base64_chars.find(char_array_4[i]);

      char_array_3[0] =
          (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] =
          ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

      for (i = 0; (i < 3); i++) ret += char_array_3[i];
      i = 0;
    }
  }

  if (i) {
    for (j = i; j < 4; j++) char_array_4[j] = 0;

    for (j = 0; j < 4; j++)
      char_array_4[j] = base64_chars.find(char_array_4[j]);

    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] =
        ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

    for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
  }

  return ret;
}

// Base64 encoding from
// https://stackoverflow.com/questions/342409/how-do-i-base64-encode-decode-in-c

static const unsigned char base64_table[65] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const unsigned char* src, size_t len) {
  unsigned char *out, *pos;
  const unsigned char *end, *in;

  size_t olen;

  olen = 4 * ((len + 2) / 3); /* 3-byte blocks to 4-byte */

  if (olen < len) return std::string(); /* integer overflow */

  std::string outStr;
  outStr.resize(olen);
  out = (unsigned char*)&outStr[0];

  end = src + len;
  in = src;
  pos = out;
  while (end - in >= 3) {
    *pos++ = base64_table[in[0] >> 2];
    *pos++ = base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)];
    *pos++ = base64_table[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
    *pos++ = base64_table[in[2] & 0x3f];
    in += 3;
  }

  if (end - in) {
    *pos++ = base64_table[in[0] >> 2];
    if (end - in == 1) {
      *pos++ = base64_table[(in[0] & 0x03) << 4];
      *pos++ = '=';
    } else {
      *pos++ = base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)];
      *pos++ = base64_table[(in[1] & 0x0f) << 2];
    }
    *pos++ = '=';
  }

  return outStr;
}

MissionReceiver* socketio_ground_communicator;

void on_connect() { socketio_ground_communicator->OnConnect(); }

void on_fail() { socketio_ground_communicator->OnFail(); }

void connect() { socketio_ground_communicator->ConnectToGround(); }

MissionReceiver::MissionReceiver()
    : phased_loop_(std::chrono::milliseconds(200),
                   std::chrono::milliseconds(0)),
      running_(false),
      last_serial_telemetry_sent_(0) {
  socketio_ground_communicator = this;

  client_.set_open_listener(on_connect);
  LOG_LINE("ground_communicator started.");
  SetFlightLoopGoal(STANDBY);

  ::std::thread ground_socket_thread(connect);
  ground_socket_thread.join();
}

void MissionReceiver::ConnectToGround() {
#ifdef UAS_AT_UCLA_DEPLOYMENT
  client_.connect("http://192.168.2.20:8081");
#else
  client_.connect("http://0.0.0.0:8081");
#endif
}

void MissionReceiver::SendTelemetry(int loop_index, int message_index) {

  double current_time =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          ::std::chrono::system_clock::now().time_since_epoch())
          .count() *
      1e-9;

  sio::message::ptr all_data = sio::object_message::create();

  auto sensors = &::src::control::loops::flight_loop_queue.sensors;
  auto status = &::src::control::loops::flight_loop_queue.status;
  auto goal = &::src::control::loops::flight_loop_queue.goal;
  auto output = &::src::control::loops::flight_loop_queue.output;

  sensors->FetchLatest();
  status->FetchLatest();
  goal->FetchLatest();
  output->FetchLatest();

  sio::message::ptr telemetry = sio::object_message::create();

  sio::message::ptr sensors_object = sio::object_message::create();
  sio::message::ptr status_object = sio::object_message::create();
  sio::message::ptr goal_object = sio::object_message::create();
  sio::message::ptr output_object = sio::object_message::create();

  std::map<std::string, sio::message::ptr>* sensors_map =
      &sensors_object->get_map();
  std::map<std::string, sio::message::ptr>* status_map =
      &status_object->get_map();
  std::map<std::string, sio::message::ptr>* goal_map = &goal_object->get_map();
  std::map<std::string, sio::message::ptr>* output_map =
      &output_object->get_map();

  bool send_sensors;
  bool send_status;
  bool send_goal;
  bool send_output;
  if (sensors->get()) {
    send_sensors = true;

    (*sensors_map)["latitude"] =
        sio::double_message::create((*sensors)->latitude);
    (*sensors_map)["longitude"] =
        sio::double_message::create((*sensors)->longitude);
    (*sensors_map)["altitude"] =
        sio::double_message::create((*sensors)->altitude);
    (*sensors_map)["relative_altitude"] =
        sio::double_message::create((*sensors)->relative_altitude);
    (*sensors_map)["heading"] =
        sio::double_message::create((*sensors)->heading);
    (*sensors_map)["velocity_x"] =
        sio::double_message::create((*sensors)->velocity_x);
    (*sensors_map)["velocity_y"] =
        sio::double_message::create((*sensors)->velocity_y);
    (*sensors_map)["velocity_z"] =
        sio::double_message::create((*sensors)->velocity_z);
    (*sensors_map)["gps_ground_speed"] =
        sio::double_message::create((*sensors)->gps_ground_speed);
    (*sensors_map)["gps_satellite_count"] =
        sio::double_message::create((*sensors)->gps_satellite_count);
    (*sensors_map)["gps_eph"] =
        sio::double_message::create((*sensors)->gps_eph);
    (*sensors_map)["gps_epv"] =
        sio::double_message::create((*sensors)->gps_epv);
    (*sensors_map)["accelerometer_x"] =
        sio::double_message::create((*sensors)->accelerometer_x);
    (*sensors_map)["accelerometer_y"] =
        sio::double_message::create((*sensors)->accelerometer_y);
    (*sensors_map)["accelerometer_z"] =
        sio::double_message::create((*sensors)->accelerometer_z);
    (*sensors_map)["gyro_x"] = sio::double_message::create((*sensors)->gyro_x);
    (*sensors_map)["gyro_y"] = sio::double_message::create((*sensors)->gyro_y);
    (*sensors_map)["gyro_z"] = sio::double_message::create((*sensors)->gyro_z);
    (*sensors_map)["absolute_pressure"] =
        sio::double_message::create((*sensors)->absolute_pressure);
    (*sensors_map)["relative_pressure"] =
        sio::double_message::create((*sensors)->relative_pressure);
    (*sensors_map)["pressure_altitude"] =
        sio::double_message::create((*sensors)->pressure_altitude);
    (*sensors_map)["temperature"] =
        sio::double_message::create((*sensors)->temperature);
    (*sensors_map)["battery_voltage"] =
        sio::double_message::create((*sensors)->battery_voltage);
    (*sensors_map)["battery_current"] =
        sio::double_message::create((*sensors)->battery_current);
    (*sensors_map)["armed"] = sio::bool_message::create((*sensors)->armed);

    ::src::control::io::AutopilotState autopilot_state =
        static_cast<::src::control::io::AutopilotState>(
            (*sensors)->autopilot_state);

    ::std::string autopilot_state_string;

    switch (autopilot_state) {
      case ::src::control::io::AutopilotState::TAKEOFF:
        autopilot_state_string = "TAKEOFF";
        break;
      case ::src::control::io::AutopilotState::HOLD:
        autopilot_state_string = "HOLD";
        break;
      case ::src::control::io::AutopilotState::OFFBOARD:
        autopilot_state_string = "OFFBOARD";
        break;
      case ::src::control::io::AutopilotState::RTL:
        autopilot_state_string = "RTL";
        break;
      case ::src::control::io::AutopilotState::LAND:
        autopilot_state_string = "LAND";
        break;
      case ::src::control::io::AutopilotState::UNKNOWN:
      default:
        autopilot_state_string = "UNKNOWN";
        break;
    }

    (*sensors_map)["autopilot_state"] =
        sio::string_message::create(autopilot_state_string);

    if(current_time - last_serial_telemetry_sent_ > 0.5) {
      // Time to send another serial telemetry message.
      ::lib::serial_comms::SerialCommsMessage message;
      message.set_latitude((*sensors)->latitude);
      message.set_longitude((*sensors)->longitude);
      message.set_altitude((*sensors)->relative_altitude);

      serial_comms_bridge_.SendData(message);

      last_serial_telemetry_sent_ = current_time;
    }
  }

  if (status->get()) {
    send_status = true;

    std::string state = ::src::control::loops::state_string.at(
        static_cast<::src::control::loops::FlightLoop::State>(
            (*status)->state));
    (*status_map)["state"] = sio::string_message::create(state);
    (*status_map)["flight_time"] =
        sio::int_message::create((*status)->flight_time);
  }

  if (goal->get()) {
    send_goal = true;

    (*goal_map)["run_mission"] =
        sio::bool_message::create((*goal)->run_mission);
    (*goal_map)["trigger_failsafe"] =
        sio::bool_message::create((*goal)->trigger_failsafe);
    (*goal_map)["trigger_throttle_cut"] =
        sio::bool_message::create((*goal)->trigger_throttle_cut);
  }

  if (output->get()) {
    send_output = true;

    (*output_map)["velocity_x"] =
        sio::double_message::create((*output)->velocity_x);
    (*output_map)["velocity_y"] =
        sio::double_message::create((*output)->velocity_y);
    (*output_map)["velocity_z"] =
        sio::double_message::create((*output)->velocity_z);

    (*output_map)["gimbal_angle"] =
        sio::double_message::create((*output)->gimbal_angle);
    (*output_map)["bomb_drop"] =
        sio::bool_message::create((*output)->bomb_drop);
    (*output_map)["alarm"] =
        sio::bool_message::create((*output)->alarm);
    (*output_map)["dslr"] =
        sio::bool_message::create((*output)->dslr);
    //  (*output_map)["velocity_control"] =
    //      sio::bool_message::create((*output)->velocity_control);
    //  (*output_map)["arm"] = sio::bool_message::create((*output)->arm);
    //  (*output_map)["disarm"] = sio::bool_message::create((*output)->disarm);
    //  (*output_map)["takeoff"] =
    //  sio::bool_message::create((*output)->takeoff);
    //  (*output_map)["land"] = sio::bool_message::create((*output)->land);
    //  (*output_map)["throttle_cut"] =
    //      sio::bool_message::create((*output)->throttle_cut);
  }

  LOG_LINE("sending telemetry: "
           << (send_sensors ? "sensors " : "") << (send_status ? "status " : "")
           << (send_goal ? "goal " : "") << (send_output ? "output " : ""));

  telemetry->get_map()["sensors"] = sensors_object;
  telemetry->get_map()["status"] = status_object;
  telemetry->get_map()["goal"] = goal_object;
  telemetry->get_map()["output"] = output_object;

  all_data->get_map()["telemetry"] = telemetry;

  ::lib::mission_manager::Mission mission =
      mission_message_queue_sender_.GetMission();
  ::std::string serialized_mission;
  mission.SerializeToString(&serialized_mission);
  const unsigned char* serialized_mission_cstr =
      reinterpret_cast<const unsigned char*>(serialized_mission.c_str());
  ::std::string mission_base64 =
      base64_encode(serialized_mission_cstr, serialized_mission.length());
  all_data->get_map()["mission"] = sio::string_message::create(mission_base64);

  all_data->get_map()["loop_index"] = sio::int_message::create(loop_index);
  all_data->get_map()["message_index"] =
      sio::int_message::create(message_index);

  client_.socket()->emit("telemetry", all_data);
}

void MissionReceiver::Run() {
  running_ = true;

  int loop_index = 0;
  int message_index = 0;

  while (running_) {
    // std::chrono::steady_clock::time_point begin =
    // std::chrono::steady_clock::now();
    RunIteration(loop_index, message_index);

    const int iterations = phased_loop_.SleepUntilNext();
    if (iterations > 1) {
      std::cout << "SKIPPED " << (iterations - 1) << " TELEMETRY ITERATIONS\n";
    }
    // std::chrono::steady_clock::time_point end=
    // std::chrono::steady_clock::now();
    // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end -
    // begin).count() << std::endl;
    message_index++;
    loop_index += iterations;
  }
}

void MissionReceiver::RunIteration(int loop_index, int message_index) {
  // SetFlightLoopGoal(GetState());
  SendTelemetry(loop_index, message_index);
}

void MissionReceiver::OnConnect() {
  LOG_LINE("Someone connected to ground_communicator");
  client_.socket()->emit("join_room", sio::string_message::create("drone"));

  client_.socket()->on(
      "drone_execute_commands",
      sio::socket::event_listener_aux(
          [&](std::string const& name, sio::message::ptr const& data,
              bool isAck, sio::message::list& ack_resp) {

            (void)name;
            (void)isAck;
            (void)ack_resp;

            ::lib::mission_manager::GroundData ground_data;
            ::lib::mission_manager::Mission* mission =
                new ::lib::mission_manager::Mission();

            ::std::string serialized_protobuf_mission = data->get_string();

            serialized_protobuf_mission =
                base64_decode(serialized_protobuf_mission);

            mission->ParseFromString(serialized_protobuf_mission);
            ground_data.set_allocated_mission(mission);

            mission_message_queue_sender_.SendData(ground_data);

            SetState("MISSION");
          }));

  client_.socket()->on(
      "interop_data",
      sio::socket::event_listener_aux(
          [&](std::string const& name, sio::message::ptr const& data,
              bool isAck, sio::message::list& ack_resp) {

            (void)name;
            (void)isAck;
            (void)ack_resp;

            ::lib::mission_manager::GroundData ground_data;
            ::lib::mission_manager::Obstacles* obstacles =
                new ::lib::mission_manager::Obstacles();

            ::std::string serialized_protobuf_obstacles = data->get_string();

            serialized_protobuf_obstacles =
                base64_decode(serialized_protobuf_obstacles);

            obstacles->ParseFromString(serialized_protobuf_obstacles);
            ground_data.set_allocated_obstacles(obstacles);
            mission_message_queue_sender_.SendData(ground_data);
          }));

  client_.socket()->on(
      "drone_set_state",
      sio::socket::event_listener_aux(
          [&](std::string const& name, sio::message::ptr const& data,
              bool isAck, sio::message::list& ack_resp) {
            (void)name;
            (void)isAck;
            (void)ack_resp;

            SetState(data->get_map()["state"]->get_string());
          }));
}

void MissionReceiver::OnFail() { ::std::cout << "socketio failed! :(\n"; }

void MissionReceiver::SetFlightLoopGoal(GoalState new_state) {
  auto last_goal = &::src::control::loops::flight_loop_queue.goal;
  last_goal->FetchLatest();

  auto flight_loop_goal_message =
      ::src::control::loops::flight_loop_queue.goal.MakeMessage();

  if (!::src::control::loops::flight_loop_queue.goal.get()) {
    // Set initial goal.
    flight_loop_goal_message->run_mission = false;
    flight_loop_goal_message->trigger_failsafe = false;
    flight_loop_goal_message->trigger_throttle_cut = false;
    flight_loop_goal_message->trigger_alarm = 0;
    flight_loop_goal_message->trigger_bomb_drop = 0;
  } else {
    // Copy past goal.
    flight_loop_goal_message->run_mission = (*last_goal)->trigger_alarm;
    flight_loop_goal_message->trigger_failsafe = (*last_goal)->trigger_failsafe;
    flight_loop_goal_message->trigger_throttle_cut =
        (*last_goal)->trigger_throttle_cut;
    flight_loop_goal_message->trigger_alarm = (*last_goal)->trigger_alarm;
  }

  double time = ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
                    ::std::chrono::system_clock::now().time_since_epoch())
                    .count() /
                1e9;

  flight_loop_goal_message->run_mission = false;

  switch (new_state) {
    case STANDBY:
      break;

    case RUN_MISSION:
      flight_loop_goal_message->run_mission = true;
      flight_loop_goal_message->trigger_failsafe = false;
      flight_loop_goal_message->trigger_throttle_cut = false;
      break;

    //  case LAND:
    //    flight_loop_goal_message->run_mission = false;
    //    flight_loop_goal_message->trigger_failsafe = false;
    //    flight_loop_goal_message->trigger_throttle_cut = false;
    //    break;

    case FAILSAFE:
      flight_loop_goal_message->run_mission = false;
      flight_loop_goal_message->trigger_failsafe = true;
      flight_loop_goal_message->trigger_throttle_cut = false;
      break;

    case THROTTLE_CUT:
      flight_loop_goal_message->run_mission = false;
      flight_loop_goal_message->trigger_failsafe = true;
      flight_loop_goal_message->trigger_throttle_cut = true;
      break;

    case TAKEOFF:
      flight_loop_goal_message->trigger_takeoff = time;
      LOG_LINE("GOT TAKEOFF @ TIME " << time);
      break;

    case HOLD:
      flight_loop_goal_message->trigger_hold = time;
      LOG_LINE("GOT HOLD @ TIME " << time);
      break;

    case OFFBOARD:
      flight_loop_goal_message->trigger_offboard = time;
      LOG_LINE("GOT OFFBOARD @ TIME " << time);
      break;

    case RTL:
      flight_loop_goal_message->trigger_rtl = time;
      LOG_LINE("GOT RTL @ TIME " << time);
      break;

    case LAND:
      flight_loop_goal_message->trigger_land = time;
      LOG_LINE("GOT LAND @ TIME " << time);
      break;

    case ARM:
      flight_loop_goal_message->trigger_arm = time;
      LOG_LINE("GOT ARM @ TIME " << time);
      break;

    case DISARM:
      flight_loop_goal_message->trigger_disarm = time;
      LOG_LINE("GOT DISARM @ TIME " << time);
      break;

    case ALARM:
      flight_loop_goal_message->trigger_alarm = time;
      LOG_LINE("GOT ALARM @ TIME " << time);
      break;

    case BOMB_DROP:
      flight_loop_goal_message->trigger_bomb_drop = time;
      LOG_LINE("GOT BOMB DROP @ TIME " << time);
      break;

    case DSLR:
      flight_loop_goal_message->trigger_dslr = time;
      LOG_LINE("GOT DSLR TRIGGER @ TIME " << time);
      break;
  }

  flight_loop_goal_message.Send();
}

void MissionReceiver::SetState(::std::string new_state_string) {
  auto sensors = &::src::control::loops::flight_loop_queue.sensors;
  auto status = &::src::control::loops::flight_loop_queue.status;
  sensors->FetchLatest();
  status->FetchLatest();

  GoalState new_state;

  if (new_state_string == "MISSION") {
    if ((*status)->state == ::src::control::loops::FlightLoop::State::LANDING &&
        (*sensors)->relative_altitude < 5.0) {
      ::std::cerr << "Cannot switch to mission: landing and at unsafe altitude."
                  << ::std::endl;
      return;
    }
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

  if ((GetState() == FAILSAFE && new_state != THROTTLE_CUT) ||
      (GetState() == THROTTLE_CUT)) {
    ::std::cerr << "Could not override higher order emergency state."
                << ::std::endl;

    return;
  }

  state_mutex_.lock();
  state_ = new_state;
  state_mutex_.unlock();

  SetFlightLoopGoal(new_state);
}

MissionReceiver::GoalState MissionReceiver::GetState() {
  state_mutex_.lock();
  GoalState state = state_;
  state_mutex_.unlock();

  return state;
}

}  // namespace ground_communicator
}  // namespace control
}  // namespace src
