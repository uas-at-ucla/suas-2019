#include "mission_receiver.h"

#include <iostream>

namespace spinny {
namespace control {
namespace mission_receiver {

MissionReceiver *socketio_mission_receiver;

void on_connect() {
  socketio_mission_receiver->OnConnect();
}

void on_fail() {
  socketio_mission_receiver->OnFail();
}

MissionReceiver::MissionReceiver() {
  socketio_mission_receiver = this;

  client_.set_open_listener(on_connect);
  client_.connect("0.0.0.0:8085");
}

void MissionReceiver::Run() {

}

void MissionReceiver::OnConnect() {
  ::std::cout << "got on_connect\n";
  client_.socket()->on("execute_commands",
           sio::socket::event_listener_aux([&](
               std::string const& name,
               sio::message::ptr const& data,
               bool isAck,
               sio::message::list &ack_resp) {
    (void)name;
    (void)data;
    (void)isAck;
    (void)ack_resp;

    ::std::cout << "got execute_commands\n";
  }));

  client_.socket()->on("set_state",
           sio::socket::event_listener_aux([&](
               std::string const& name,
               sio::message::ptr const& data,
               bool isAck,
               sio::message::list &ack_resp) {
    (void)name;
    (void)data;
    (void)isAck;
    (void)ack_resp;

    ::std::cout << "got set_state\n";
  }));
}

void MissionReceiver::OnFail() {
  ::std::cout << "socketio failed! :(\n";
}

}  // namespace mission_receiver
}  // namespace control
}  // namespace spinny

