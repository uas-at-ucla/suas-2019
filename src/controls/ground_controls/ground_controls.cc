#include "ground_controls.h"

namespace src {
namespace controls {
namespace ground_controls {

GroundControls *socketio_ground_controls;

void on_connect() { socketio_ground_controls->OnConnect(); }

void on_fail() { socketio_ground_controls->OnFail(); }

GroundControls::GroundControls(int argc, char **argv) :
    running_(false),
    ros_node_handle_(),
    sensors_subscriber_(ros_node_handle_.subscribe(
        kRosSensorsTopic, kRosMessageQueueSize,
        &GroundControls::SensorsReceived, this, ros::TransportHints().udp())),
    drone_program_subscriber_(ros_node_handle_.subscribe(
        kRosDroneProgramTopic, kRosMessageQueueSize,
        &GroundControls::DroneProgramReceived, this)),
    mission_status_subscriber_(ros_node_handle_.subscribe(
        kRosMissionStatusTopic, kRosMessageQueueSize,
        &GroundControls::MissionStatusReceived, this)),
    drone_program_publisher_(
        ros_node_handle_.advertise<
            ::src::controls::ground_controls::timeline::DroneProgram>(
            kRosDroneProgramTopic, kRosMessageQueueSize)),
    mission_status_publisher_(ros_node_handle_.advertise<::std_msgs::String>(
        kRosMissionStatusTopic, kRosMessageQueueSize)),
    udp_connection_("tcp://127.0.0.1:6005", 1),
    rfd900_connection_("/dev/ttyUSB0", B57600, 0), // TODO
    phased_loop_(1e2),
    drone_program_success_(false) {

  // Connect proto_receiver_
  udp_connection_.Connect();

  socketio_ground_controls = this;
  client_.set_open_listener(on_connect);
  client_.set_fail_listener(on_fail);

  if (argc >= 2) {
    ::std::string ip(argv[1]);
    client_.connect("http://" + ip + ":8081");
  } else {
    client_.connect("http://localhost:8081");
  }
}

void GroundControls::SendSensorsToServer(
    const ::src::controls::Sensors &sensors, bool rfd900) {
  if (sensors.IsInitialized()) { // IsInitialized checks that all required
                                 // fields are present
    ::std::string sensors_serialized;
    sensors.SerializeToString(&sensors_serialized);

    // ::std::cout << "sensors: " << sensors.DebugString() << "\n";

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

void GroundControls::ReadUDP() {
  running_ = true;

  while (running_) {
    // Get the connection's uas messages
    ::src::controls::UasMessage uas_message1;
    ::src::controls::UasMessage uas_message2;
    bool udp_res = udp_connection_.GetLatestProto(uas_message1);

    // Check for either availability
    if (!udp_res) {
      ::std::cout << "Did not get upd connection" << ::std::endl;
      return;
    } else {
      // TODO: check which one was received earliest
      ::std::cout << "Got udp connection" << ::std::endl;
    }

    phased_loop_.SleepUntilNext();
  }
}

void GroundControls::DroneProgramReceived(
    const ::src::controls::ground_controls::timeline::DroneProgram
        drone_program) {
  ::std::string serialized_drone_program;
  drone_program.SerializeToString(&serialized_drone_program);
  serialized_drone_program =
      ::lib::base64_tools::Encode(serialized_drone_program);
  client_.socket("ground-controls")
      ->emit("UPLOADED_DRONE_PROGRAM", serialized_drone_program);
}

void GroundControls::MissionStatusReceived(
    const ::std_msgs::String mission_status) {
  client_.socket("ground-controls")
      ->emit("MISSION_STATUS", mission_status.data);
}

void GroundControls::OnConnect() {
  client_.socket("ground-controls")
      ->on(
          "COMPILE_GROUND_PROGRAM",
          ::sio::socket::event_listener_aux(
              [&](::std::string const &name, ::sio::message::ptr const &data,
                  bool isAck, ::sio::message::list &ack_resp) {
                (void)name;
                (void)isAck;
                (void)ack_resp;

                // Get the encoded serialized GroundProgram protobuf from the
                // payload.
                ::std::string serialized_protobuf_mission = data->get_string();

                // Use base64 to decode the string data into raw bytes.
                serialized_protobuf_mission =
                    ::lib::base64_tools::Decode(serialized_protobuf_mission);

                // Deserialize the GroundProgram protobuf from the serialized
                // data.
                ::src::controls::ground_controls::timeline::GroundProgram
                    ground_program;
                if (!ground_program.ParseFromString(
                        serialized_protobuf_mission)) {
                  ::std::cout << "drone program compilation failure: failed to "
                                 "deserialize protobuf data."
                              << ::std::endl;
                  return;
                }

                // Compile the GroundProgram into a DroneProgram.
                drone_program_ = ground2drone_visitor_.Process(&ground_program);
                drone_program_success_ = drone_program_.IsInitialized();

                if (!drone_program_success_) {
                  ::std::cout
                      << "drone program compilation failure: Could not compile "
                         "GroundProgram"
                      << ::std::endl;
                  client_.socket("ground-controls")
                      ->emit("MISSION_COMPILE_ERROR");
                  return;
                }

                ::std::cout << "compiled drone program:\n"
                            << drone_program_.DebugString() << "\n";

                ::std::string serialized_drone_program;
                drone_program_.SerializeToString(&serialized_drone_program);
                serialized_drone_program =
                    ::lib::base64_tools::Encode(serialized_drone_program);
                client_.socket("ground-controls")
                    ->emit("COMPILED_DRONE_PROGRAM", serialized_drone_program);
              }));

  client_.socket("ground-controls")
      ->on("UPLOAD_MISSION",
           ::sio::socket::event_listener_aux(
               [&](::std::string const &name, ::sio::message::ptr const &data,
                   bool isAck, ::sio::message::list &ack_resp) {
                 (void)name;
                 (void)data;
                 (void)isAck;
                 (void)ack_resp;
                 if (drone_program_success_) {
                   ::std::cout << "Uploading mission!\n";
                   drone_program_publisher_.publish(drone_program_);
                 } else {
                   ::std::cout
                       << "Oops, can't upload mission. The drone program "
                          "hasn't been successfully compiled.\n";
                 }
               }));

  client_.socket("ground-controls")
      ->on("RUN_MISSION",
           ::sio::socket::event_listener_aux(
               [&](::std::string const &name, ::sio::message::ptr const &data,
                   bool isAck, ::sio::message::list &ack_resp) {
                 (void)name;
                 (void)data;
                 (void)isAck;
                 (void)ack_resp;
                 ::std_msgs::String mission_status;
                 mission_status.data = "RUN_MISSION";
                 mission_status_publisher_.publish(mission_status);
               }));

  client_.socket("ground-controls")
      ->on("PAUSE_MISSION",
           ::sio::socket::event_listener_aux(
               [&](::std::string const &name, ::sio::message::ptr const &data,
                   bool isAck, ::sio::message::list &ack_resp) {
                 (void)name;
                 (void)data;
                 (void)isAck;
                 (void)ack_resp;
                 ::std_msgs::String mission_status;
                 mission_status.data = "PAUSE_MISSION";
                 mission_status_publisher_.publish(mission_status);
               }));

  client_.socket("ground-controls")
      ->on("END_MISSION",
           ::sio::socket::event_listener_aux(
               [&](::std::string const &name, ::sio::message::ptr const &data,
                   bool isAck, ::sio::message::list &ack_resp) {
                 (void)name;
                 (void)data;
                 (void)isAck;
                 (void)ack_resp;
                 ::std_msgs::String mission_status;
                 mission_status.data = "END_MISSION";
                 mission_status_publisher_.publish(mission_status);
               }));
}

void GroundControls::OnFail() { ::std::cout << "socketio failed! :(\n"; }

GroundControls::~GroundControls() { rfd900_connection_.Quit(); }

} // namespace ground_controls
} // namespace controls
} // namespace src
