#include "ground_controls.h"

namespace src {
namespace controls {
namespace ground_controls {

GroundControls *socketio_ground_controls;

void on_connect() { socketio_ground_controls->OnConnect(); }

void on_fail() { socketio_ground_controls->OnFail(); }

void connect() { socketio_ground_controls->ConnectToGround(); }

GroundControls::GroundControls() :
    udp_connection_("udp://:*5555", 1),
    rfd900_connection_("udp://:*5555", 57600, 0),
    phased_loop_(1e2),
    running_(false) {

  socketio_ground_controls = this;

  client_.set_open_listener(on_connect);

  // Connect proto_receiver_
  udp_connection_.Connect();

  ::std::thread ground_socket_thread(connect);
  ground_socket_thread.join();
}

void GroundControls::ConnectToGround() {
#ifdef UAS_AT_UCLA_DEPLOYMENT
  client_.connect("http://192.168.2.20:8081");
#else
  client_.connect("http://192.168.2.20:8081");
#endif
}

void GroundControls::StateReceived(const ::mavros_msgs::State state) {
  // TODO(comran): Fix this.
  (void)state;
  // sensors_.set_mode(state.mode);
}

void GroundControls::Run() {
  running_ = true;

  while (running_) {
    RunIteration();

    phased_loop_.SleepUntilNext();
  }
}

void GroundControls::RunIteration() {
  ::ros::spinOnce();

  // Get the connection's uas messages
  ::src::controls::UasMessage uas_message1;
  ::src::controls::UasMessage uas_message2;
  bool rfd900_res = rfd900_connection_.GetLatestProto(uas_message1); 
  bool udp_connection_res = udp_connection_.GetLatestProto(uas_message2);

  // Check for either availability
  if (!rfd900_res && !udp_connection_res) {
    return;
  } else if (!rfd900_res && udp_connection_res) {
    ::std::cout << "Got udp connection and not rfd900" << ::std::endl;
  } else if (rfd900_res && !udp_connection_res) {
    ::std::cout << "Got rfd900 and not udp connection" << ::std::endl;
  } else {
    // TODO: check which one was received earliest
    ::std::cout << "Got both rfd900 and udp connection" << ::std::endl;
  }
}

void GroundControls::OnConnect() {
  client_.socket("drone")->on(
      "compile_ground_program",
      ::sio::socket::event_listener_aux(
          [&](::std::string const &name, ::sio::message::ptr const &data,
              bool isAck, ::sio::message::list &ack_resp) {
            (void)name;
            (void)isAck;
            (void)ack_resp;
            (void)data;

            // TODO: Compile ground program into drone program.
          }));

  
}

void GroundControls::OnFail() { ::std::cout << "socketio failed! :(\n"; }

GroundControls::~GroundControls() {
  rfd900_connection_.Quit();
}

} // namespace ground_controls
} // namespace controls
} // namespace src
