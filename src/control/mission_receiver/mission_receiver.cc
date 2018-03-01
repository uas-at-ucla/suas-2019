#include "mission_receiver.h"

namespace spinny {
namespace control {
namespace mission_receiver {

MissionReceiver* socketio_mission_receiver;

void on_connect() { socketio_mission_receiver->OnConnect(); }

void on_fail() { socketio_mission_receiver->OnFail(); }

void connect() { socketio_mission_receiver->ConnectToGround(); }

MissionReceiver::MissionReceiver()
    : context_(1),
      mission_command_stream_(context_, ZMQ_REP),
      phased_loop_(std::chrono::milliseconds(10), std::chrono::milliseconds(0)),
      running_(false) {
  mission_command_stream_.bind("ipc:///tmp/mission_command_stream.ipc");

  socketio_mission_receiver = this;

  client_.set_open_listener(on_connect);

  ::std::thread ground_socket_thread(connect);
  ground_socket_thread.join();
}

void MissionReceiver::ConnectToGround() {
  client_.connect("http://0.0.0.0:8085");
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
  auto flight_loop_goal_message =
      ::spinny::control::loops::flight_loop_queue.goal.MakeMessage();

  flight_loop_goal_message->run_mission = true;
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
      "execute_commands",
      sio::socket::event_listener_aux([&](
          std::string const& name, sio::message::ptr const& data, bool isAck,
          sio::message::list& ack_resp) {

        (void)name;
        (void)isAck;
        (void)ack_resp;

        ::spinny::controls::mission_receiver::Mission mission;

        for (size_t i = 0; i < data->get_vector().size(); i++) {
          ::spinny::controls::mission_receiver::Command* cmd =
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
      }));

  client_.socket()->on(
      "set_state", sio::socket::event_listener_aux([&](
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

}  // namespace mission_receiver
}  // namespace control
}  // namespace spinny
