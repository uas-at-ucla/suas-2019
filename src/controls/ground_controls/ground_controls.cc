#include "ground_controls.h"

namespace src {
namespace controls {
namespace ground_controls {

GroundControls *socketio_ground_controls;

void on_connect() { socketio_ground_controls->OnConnect(); }

void on_fail() { socketio_ground_controls->OnFail(); }

GroundControls::GroundControls() :
    running_(false),
    ros_node_handle_(),
    sensors_subscriber_(ros_node_handle_.subscribe(
        io::kRosSensorsTopic, io::kRosMessageQueueSize,
        &GroundControls::SensorsReceived, this)),
    // udp_connection_("udp://:*5555", 1),
    rfd900_connection_("/dev/ttyUSB0", B57600, 0), // TODO
    phased_loop_(1e2) {

  // Connect proto_receiver_
  // udp_connection_.Connect();

  socketio_ground_controls = this;
  client_.set_open_listener(on_connect);
  client_.set_fail_listener(on_fail);

#ifdef UAS_AT_UCLA_DEPLOYMENT
  client_.connect("http://localhost:8081");
#else
  client_.connect("http://192.168.2.20:8081");
#endif
}

void GroundControls::SendSensorsToServer(
    const ::src::controls::Sensors &sensors, bool rfd900) {
  if (sensors.IsInitialized()) { // IsInitialized checks that all required
                                 // fields are present
    ::std::string sensors_serialized;
    sensors.SerializeToString(&sensors_serialized);

    ::std::cout << "sensors: " << sensors.DebugString() << "\n";

    if (client_.opened()) {
      client_.socket("ground-controls")
          ->emit(rfd900 ? "SENSORS_RFD900" : "SENSORS",
                 ::sio::string_message::create(
                     ::lib::base64_tools::Encode(sensors_serialized)));
    }
  }
}

void GroundControls::SensorsReceived(const ::src::controls::Sensors sensors) {
  SendSensorsToServer(sensors, false);
}

void GroundControls::ReadRFD900() {
  running_ = true;

  while (running_) {
    // Get the connection's uas messages
    ::src::controls::UasMessage uas_message1;
    ::src::controls::UasMessage uas_message2;
    bool rfd900_res = rfd900_connection_.GetLatestProto(uas_message1);
    bool udp_connection_res =
        false; // udp_connection_.GetLatestProto(uas_message2);

    // Check for either availability
    if (!rfd900_res && !udp_connection_res) {
      return;
    } else if (!rfd900_res && udp_connection_res) {
      ::std::cout << "Got udp connection and not rfd900" << ::std::endl;
    } else if (rfd900_res && !udp_connection_res) {
      if (!::ros::master::check()) { // if ros is not available
        SendSensorsToServer(uas_message1.sensors(), true);
      }
      ::std::cout << "Got rfd900 and not udp connection" << ::std::endl;
    } else {
      // TODO: check which one was received earliest
      ::std::cout << "Got both rfd900 and udp connection" << ::std::endl;
    }

    phased_loop_.SleepUntilNext();
  }
}

void GroundControls::OnConnect() {
  client_.socket("drone")->on(
      "compile_ground_program",
      ::sio::socket::event_listener_aux([&](::std::string const &name,
                                            ::sio::message::ptr const &data,
                                            bool isAck,
                                            ::sio::message::list &ack_resp) {
        (void)name;
        (void)isAck;
        (void)ack_resp;
        (void)data;

        // TODO: Compile ground program into drone program.
        // ::lib::mission_manager::GroundData ground_data;
        // ::lib::mission_manager::Mission *mission =
        //     new ::lib::mission_manager::Mission();
        // ::std::string serialized_protobuf_mission = data->get_string();

        // serialized_protobuf_mission =
        //     ::lib::base64_tools::Decode(serialized_protobuf_mission);

        // mission->ParseFromString(serialized_protobuf_mission);
        // ground_data.set_allocated_mission(mission);

        // mission_message_queue_sender_.SendData(ground_data);
        ::src::controls::ground_controls::timeline::GroundProgram
            *ground_program =
                new ::src::controls::ground_controls::timeline::GroundProgram();

        ::std::string serialized_protobuf_mission = data->get_string();

        serialized_protobuf_mission =
            ::lib::base64_tools::Decode(serialized_protobuf_mission);

        ground_program->ParseFromString(serialized_protobuf_mission);

        ::src::controls::ground_controls::timeline::DroneProgram drone_program;
        bool success =
            ground2drone_visitor_.Process(ground_program, drone_program);
        if (success) {
          ::std::string serialized_drone_program;
          drone_program.SerializeToString(&serialized_drone_program);
          client_.socket("drone")->emit("compiled_drone_program",
                                        serialized_drone_program);
        } else {
          ::std::cout << "drone program compilation failure" << ::std::endl;
        }
      }));
}

void GroundControls::OnFail() { ::std::cout << "socketio failed! :(\n"; }

GroundControls::~GroundControls() { rfd900_connection_.Quit(); }

} // namespace ground_controls
} // namespace controls
} // namespace src
