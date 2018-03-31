#include "mission_message_queue.h"

namespace lib {
namespace mission_message_queue {

// Sender //////////////////////////////////////////////////////////////////////
MissionMessageQueueSender::MissionMessageQueueSender() {}

void MissionMessageQueueSender::operator()() {
  // Listen to ZMQ message queue and update MissionManager with any new missions
  // that come along.
  ::zmq::context_t context(1);
  ::zmq::socket_t ground_communicator_stream(context, ZMQ_REP);
  ground_communicator_stream.connect("ipc:///tmp/mission_command_stream.ipc");

  while (run_) {
    ::zmq::message_t mission_message;

    ground_communicator_stream.recv(&mission_message);
    ground_communicator_stream.send(mission_message);

    ::std::string mission_message_string(
        static_cast<char *>(mission_message.data()), mission_message.size());

    ::lib::mission_message_queue::Mission mission_protobuf;
    mission_protobuf.ParseFromString(mission_message_string);
  }
}

::std::vector<::std::shared_ptr<::lib::MissionCommand>>
MissionMessageQueueSender::ParseMissionProtobuf(
    ::lib::mission_message_queue::Mission mission_protobuf) {
  ::std::vector<::std::shared_ptr<::lib::MissionCommand>> new_commands;

  // Iterate through all commands in the protobuf and convert them to our C++
  // objects for commands.
  for (::lib::mission_message_queue::Command cmd_protobuf :
       mission_protobuf.commands()) {
    if (cmd_protobuf.type() == "goto") {
      new_commands.push_back(::std::make_shared<::lib::MissionCommandGoto>(
          new ::lib::MissionCommandGoto(cmd_protobuf.latitude(),
                                        cmd_protobuf.longitude(),
                                        cmd_protobuf.altitude())));

    } else if (cmd_protobuf.type() == "bomb") {
      new_commands.push_back(::std::make_shared<::lib::MissionCommandBombDrop>(
          new ::lib::MissionCommandBombDrop()));

    } else {
      // Return an empty command protobuf if we encounter any parsing errors
      // with the mission given.
      ::std::cerr << "ERROR: Invalid command type " << cmd_protobuf.type()
                  << ::std::endl;

      return ::std::vector<::std::shared_ptr<::lib::MissionCommand>>();
    }
  }

  return new_commands;
}

// Receiver ////////////////////////////////////////////////////////////////////
MissionMessageQueueReceiver::MissionMessageQueueReceiver(
    ::lib::MissionManager *mission_manager)
    : mission_manager_(mission_manager) {}

void MissionMessageQueueReceiver::operator()() {
  // Listen to ZMQ message queue and update MissionManager with any new missions
  // that come along.
  ::zmq::context_t context(1);
  ::zmq::socket_t ground_communicator_stream(context, ZMQ_REP);
  ground_communicator_stream.connect("ipc:///tmp/mission_command_stream.ipc");

  while (run_) {
    ::zmq::message_t mission_message;

    ground_communicator_stream.recv(&mission_message);
    ground_communicator_stream.send(mission_message);

    ::std::string mission_message_string(
        static_cast<char *>(mission_message.data()), mission_message.size());

    ::lib::mission_message_queue::Mission mission_protobuf;
    mission_protobuf.ParseFromString(mission_message_string);

    mission_manager_->AddCommands(ParseMissionProtobuf(mission_protobuf));
  }
}

::std::vector<::std::shared_ptr<::lib::MissionCommand>>
MissionMessageQueueReceiver::ParseMissionProtobuf(
    ::lib::mission_message_queue::Mission mission_protobuf) {
  ::std::vector<::std::shared_ptr<::lib::MissionCommand>> new_commands;

  // Iterate through all commands in the protobuf and convert them to our C++
  // objects for commands.
  for (::lib::mission_message_queue::Command cmd_protobuf :
       mission_protobuf.commands()) {
    if (cmd_protobuf.type() == "goto") {
      new_commands.push_back(::std::make_shared<::lib::MissionCommandGoto>(
          new ::lib::MissionCommandGoto(cmd_protobuf.latitude(),
                                        cmd_protobuf.longitude(),
                                        cmd_protobuf.altitude())));

    } else if (cmd_protobuf.type() == "bomb") {
      new_commands.push_back(::std::make_shared<::lib::MissionCommandBombDrop>(
          new ::lib::MissionCommandBombDrop()));

    } else {
      // Return an empty command protobuf if we encounter any parsing errors
      // with the mission given.
      ::std::cerr << "ERROR: Invalid command type " << cmd_protobuf.type()
                  << ::std::endl;

      return ::std::vector<::std::shared_ptr<::lib::MissionCommand>>();
    }
  }

  return new_commands;
}

}  // mission_message_queue
}  // namespace lib
