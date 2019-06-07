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
    sensors_subscriber_(
        ros_node_handle_.subscribe(kRosSensorsTopic, kRosMessageQueueSize,
                                   &GroundControls::SensorsReceived, this)),
    output_subscriber_(
        ros_node_handle_.subscribe(kRosOutputTopic, kRosMessageQueueSize,
                                   &GroundControls::OutputReceived, this)),
    drone_program_subscriber_(ros_node_handle_.subscribe(
        kRosDroneProgramTopic, kRosMessageQueueSize,
        &GroundControls::DroneProgramReceived, this)),
    drone_program_publisher_(
        ros_node_handle_.advertise<
            ::src::controls::ground_controls::timeline::DroneProgram>(
            kRosDroneProgramTopic, kRosMessageQueueSize)),
    droppy_command_subscriber_(ros_node_handle_.subscribe(
        kRosMissionStatusTopic, kRosMessageQueueSize,
        &GroundControls::DroppyCommandReceived, this)),
    droppy_command_publisher_(ros_node_handle_.advertise<::std_msgs::String>(
        kRosMissionStatusTopic, kRosMessageQueueSize)),
    gimbal_publisher_(ros_node_handle_.advertise<::std_msgs::Float32>(
        kRosGimbalTopic, kRosMessageQueueSize, true)),
    deployment_motor_publisher_(ros_node_handle_.advertise<::std_msgs::Float32>(
        kRosDeploymentMotorTopic, kRosMessageQueueSize, true)),
    latch_publisher_(ros_node_handle_.advertise<::std_msgs::Bool>(
        kRosLatchTopic, kRosMessageQueueSize, true)),
    hotwire_publisher_(ros_node_handle_.advertise<::std_msgs::Bool>(
        kRosHotwireTopic, kRosMessageQueueSize, true)),
    gimbal_subscriber_(
        ros_node_handle_.subscribe(kRosGimbalTopic, kRosMessageQueueSize,
                                   &GroundControls::GimbalSetpoint, this)),
    deployment_motor_subscriber_(ros_node_handle_.subscribe(
        kRosDeploymentMotorTopic, kRosMessageQueueSize,
        &GroundControls::DeploymentMotorSetpoint, this)),
    latch_subscriber_(
        ros_node_handle_.subscribe(kRosLatchTopic, kRosMessageQueueSize,
                                   &GroundControls::LatchSetpoint, this)),
    hotwire_subscriber_(
        ros_node_handle_.subscribe(kRosHotwireTopic, kRosMessageQueueSize,
                                   &GroundControls::HotwireSetpoint, this)),
    rfd900_connection_("/dev/ttyUSB1", B57600, 0), // TODO
    phased_loop_(1e2),
    drone_program_success_(false) {

  // Connect proto_receiver_

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
  (void)sensors;
  // SendSensorsToServer(sensors, false);
}

void GroundControls::OutputReceived(const ::src::controls::Output output) {
  if (output.IsInitialized()) {
    ::std::string output_serialized;
    output.SerializeToString(&output_serialized);
    if (client_.opened()) {
      client_.socket("ground-controls")
          ->emit("OUTPUT", ::sio::string_message::create(
                               ::lib::base64_tools::Encode(output_serialized)));
    }
  }
}

void GroundControls::ReadRFD900() {
  running_ = true;

  while (running_) {
    // Get the connection's uas messages
    ::src::controls::UasMessage uas_message1;
    bool rfd900_res = rfd900_connection_.GetLatestProto(uas_message1);

    if (rfd900_res) {
      ROS_INFO("Got RFD900: lat(%f) lng(%f) alt(%f) hdg(%f)",
               uas_message1.sensors().latitude(),
               uas_message1.sensors().longitude(),
               uas_message1.sensors().relative_altitude(),
               uas_message1.sensors().heading());
      SendSensorsToServer(uas_message1.sensors(), true);
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
  if (client_.opened())
    client_.socket("ground-controls")
        ->emit("UPLOADED_DRONE_PROGRAM", serialized_drone_program);
}

void GroundControls::DroppyCommandReceived(
    const ::std_msgs::String droppy_command) {
  if (client_.opened())
    client_.socket("ground-controls")
        ->emit("DROPPY_COMMAND_RECEIVED", droppy_command.data);
}

void GroundControls::GimbalSetpoint(const ::std_msgs::Float32 gimbal_setpoint) {
  if (client_.opened())
    client_.socket("ground-controls")
        ->emit("GIMBAL_SETPOINT",
               ::sio::double_message::create(gimbal_setpoint.data));
}

void GroundControls::DeploymentMotorSetpoint(
    const ::std_msgs::Float32 deployment_motor_setpoint) {
  if (client_.opened())
    client_.socket("ground-controls")
        ->emit("DEPLOYMENT_MOTOR_SETPOINT",
               ::sio::double_message::create(deployment_motor_setpoint.data));
}

void GroundControls::LatchSetpoint(const ::std_msgs::Bool latch_setpoint) {
  if (client_.opened())
    client_.socket("ground-controls")
        ->emit("LATCH_SETPOINT",
               ::sio::bool_message::create(latch_setpoint.data));
}

void GroundControls::HotwireSetpoint(const ::std_msgs::Bool hotwire_setpoint) {
  if (client_.opened())
    client_.socket("ground-controls")
        ->emit("HOTWIRE_SETPOINT",
               ::sio::bool_message::create(hotwire_setpoint.data));
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
                  if (client_.opened())
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
                if (client_.opened())
                  client_.socket("ground-controls")
                      ->emit("COMPILED_DRONE_PROGRAM",
                             serialized_drone_program);
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
      ->on("CHANGE_DROPPY_STATE",
           ::sio::socket::event_listener_aux(
               [&](::std::string const &name, ::sio::message::ptr const &data,
                   bool isAck, ::sio::message::list &ack_resp) {
                 (void)name;
                 (void)isAck;
                 (void)ack_resp;
                 ::std_msgs::String droppy_command;
                 droppy_command.data = data->get_string();
                 droppy_command_publisher_.publish(droppy_command);
               }));

  client_.socket("ground-controls")
      ->on("GIMBAL_SETPOINT",
           ::sio::socket::event_listener_aux(
               [&](::std::string const &name, ::sio::message::ptr const &data,
                   bool isAck, ::sio::message::list &ack_resp) {
                 (void)name;
                 (void)isAck;
                 (void)ack_resp;
                 ::std_msgs::Float32 gimbal_setpoint;
                 gimbal_setpoint.data = data->get_double();
                 ::std::cout << "got gimbal setpoint from ground\n";
                 gimbal_publisher_.publish(gimbal_setpoint);
               }));

  client_.socket("ground-controls")
      ->on("DEPLOYMENT_MOTOR_SETPOINT",
           ::sio::socket::event_listener_aux(
               [&](::std::string const &name, ::sio::message::ptr const &data,
                   bool isAck, ::sio::message::list &ack_resp) {
                 (void)name;
                 (void)isAck;
                 (void)ack_resp;
                 ::std_msgs::Float32 deployment_motor_setpoint;
                 deployment_motor_setpoint.data = data->get_double();
                 deployment_motor_publisher_.publish(deployment_motor_setpoint);
               }));

  client_.socket("ground-controls")
      ->on("LATCH_SETPOINT",
           ::sio::socket::event_listener_aux(
               [&](::std::string const &name, ::sio::message::ptr const &data,
                   bool isAck, ::sio::message::list &ack_resp) {
                 (void)name;
                 (void)isAck;
                 (void)ack_resp;
                 ::std_msgs::Bool latch_setpoint;
                 latch_setpoint.data = data->get_bool();
                 latch_publisher_.publish(latch_setpoint);
               }));

  client_.socket("ground-controls")
      ->on("HOTWIRE_SETPOINT",
           ::sio::socket::event_listener_aux(
               [&](::std::string const &name, ::sio::message::ptr const &data,
                   bool isAck, ::sio::message::list &ack_resp) {
                 (void)name;
                 (void)isAck;
                 (void)ack_resp;
                 ::std_msgs::Bool hotwire_setpoint;
                 hotwire_setpoint.data = data->get_bool();
                 hotwire_publisher_.publish(hotwire_setpoint);
               }));
}

void GroundControls::OnFail() { ::std::cout << "socketio failed! :(\n"; }

GroundControls::~GroundControls() { rfd900_connection_.Quit(); }

} // namespace ground_controls
} // namespace controls
} // namespace src
