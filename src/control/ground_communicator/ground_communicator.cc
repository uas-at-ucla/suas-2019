#include "ground_communicator.h"

#include <iomanip>

namespace src {
namespace control {
namespace ground_communicator {

MissionReceiver* socketio_ground_communicator;

void on_connect() { socketio_ground_communicator->OnConnect(); }

void on_fail() { socketio_ground_communicator->OnFail(); }

void connect() { socketio_ground_communicator->ConnectToGround(); }

MissionReceiver::MissionReceiver()
    : context_(1),
      mission_command_stream_(context_, ZMQ_REP),
      phased_loop_(std::chrono::milliseconds(10), std::chrono::milliseconds(0)),
      running_(false),
      count_(0) {
  mission_command_stream_.bind("ipc:///tmp/mission_command_stream.ipc");

  socketio_ground_communicator = this;

  client_.set_open_listener(on_connect);

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

void MissionReceiver::SendTelemetry() {
  ::std::cout << "Sending:" << ::std::endl;

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

  if (sensors->get()) {
    ::std::cout << "Sensors" << ::std::endl;
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
    (*sensors_map)["autopilot_state"] =
        sio::int_message::create((*sensors)->autopilot_state);
  }

  if (status->get()) {
    ::std::cout << "Status" << ::std::endl;
    std::string state = ::src::control::loops::state_string.at(
        static_cast<::src::control::loops::FlightLoop::State>(
            (*status)->state));
    (*status_map)["state"] = sio::string_message::create(state);
    (*status_map)["flight_time"] = 
        sio::int_message::create((*status)->flight_time);
  }

  if (goal->get()) {
    ::std::cout << "Goal" << ::std::endl;
    (*goal_map)["run_mission"] =
        sio::bool_message::create((*goal)->run_mission);
    (*goal_map)["trigger_failsafe"] =
        sio::bool_message::create((*goal)->trigger_failsafe);
    (*goal_map)["trigger_throttle_cut"] =
        sio::bool_message::create((*goal)->trigger_throttle_cut);
  }

  if (output->get()) {
    ::std::cout << "Output" << ::std::endl;
    (*output_map)["velocity_x"] =
        sio::double_message::create((*output)->velocity_x);
    (*output_map)["velocity_y"] =
        sio::double_message::create((*output)->velocity_y);
    (*output_map)["velocity_z"] =
        sio::double_message::create((*output)->velocity_z);
    (*output_map)["velocity_control"] =
        sio::bool_message::create((*output)->velocity_control);
    (*output_map)["arm"] = sio::bool_message::create((*output)->arm);
    (*output_map)["disarm"] = sio::bool_message::create((*output)->disarm);
    (*output_map)["takeoff"] = sio::bool_message::create((*output)->takeoff);
    (*output_map)["land"] = sio::bool_message::create((*output)->land);
    (*output_map)["throttle_cut"] =
        sio::bool_message::create((*output)->throttle_cut);
  }

  ::std::cout << ::std::endl;

  telemetry->get_map()["sensors"] = sensors_object;
  telemetry->get_map()["status"] = status_object;
  telemetry->get_map()["goal"] = goal_object;
  telemetry->get_map()["output"] = output_object;

  client_.socket()->emit("telemetry", telemetry);
}

void MissionReceiver::SendTelemetryPeriodic() {
  if (count_++ % 10) return;
  SendTelemetry();
}

void MissionReceiver::Run() {
  running_ = true;

  while (running_) {
    RunIteration();

    const int iterations = phased_loop_.SleepUntilNext();
    if (iterations < 0) {
      std::cout << "SKIPPED ITERATIONS\n";
    }
  }
}

void MissionReceiver::RunIteration() {
  SendTelemetryPeriodic();

  auto flight_loop_goal_message =
      ::src::control::loops::flight_loop_queue.goal.MakeMessage();

  flight_loop_goal_message->run_mission = false;
  flight_loop_goal_message->trigger_failsafe = false;
  flight_loop_goal_message->trigger_throttle_cut = false;

  switch (GetState()) {
    case RUN_MISSION:
      flight_loop_goal_message->run_mission = true;
      break;

    case LAND:
      flight_loop_goal_message->run_mission = false;
      break;

    case FAILSAFE:
      flight_loop_goal_message->trigger_failsafe = true;
      break;

    case THROTTLE_CUT:
      flight_loop_goal_message->trigger_throttle_cut = true;
      break;
  }

  flight_loop_goal_message.Send();
}

void MissionReceiver::OnConnect() {
  ::std::cout << "========================got on_connect\n";

  client_.socket()->on(
      "drone_execute_commands",
      sio::socket::event_listener_aux([&](
          std::string const& name, sio::message::ptr const& data, bool isAck,
          sio::message::list& ack_resp) {

        (void)name;
        (void)isAck;
        (void)ack_resp;

        ::src::controls::ground_communicator::Mission mission;

        for (size_t i = 0; i < data->get_vector().size(); i++) {
          ::src::controls::ground_communicator::Command* cmd =
              mission.add_commands();

          cmd->set_type(
              data->get_vector()[i]->get_map()["command_type"]->get_string());
          cmd->set_latitude(
              data->get_vector()[i]->get_map()["lat"]->get_double());
          cmd->set_longitude(
              data->get_vector()[i]->get_map()["lng"]->get_double());
          cmd->set_altitude(
              data->get_vector()[i]->get_map()["alt"]->get_double());
        }

        ::std::string output;
        mission.SerializeToString(&output);

        zmq::message_t reply(output.size());
        memcpy((void*)reply.data(), output.c_str(), output.size());

        try {
          mission_command_stream_.send(reply);
        } catch (...) {
          ::std::cerr
              << "Could not send mission to loop. Is the loop running?\n";
        }

        SetState("MISSION");
      }));

  client_.socket()->on(
      "drone_set_state", sio::socket::event_listener_aux([&](
                       std::string const& name, sio::message::ptr const& data,
                       bool isAck, sio::message::list& ack_resp) {
        (void)name;
        (void)isAck;
        (void)ack_resp;

        SetState(data->get_map()["state"]->get_string());
      }));
}

void MissionReceiver::OnFail() { ::std::cout << "socketio failed! :(\n"; }

void MissionReceiver::SetState(::std::string new_state_string) {
  GoalState new_state;

  if (new_state_string == "MISSION") {
    new_state = RUN_MISSION;
  } else if (new_state_string == "LAND") {
    new_state = LAND;
  } else if (new_state_string == "FAILSAFE") {
    new_state = FAILSAFE;
  } else if (new_state_string == "THROTTLE CUT") {
    new_state = THROTTLE_CUT;
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
