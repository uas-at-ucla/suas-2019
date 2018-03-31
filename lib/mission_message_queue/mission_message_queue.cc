#include "mission_message_queue.h"

namespace lib {
namespace mission_message_queue {

// Sender //////////////////////////////////////////////////////////////////////
MissionMessageQueueSender::MissionMessageQueueSender()
    : context_(1), socket_(context_, ZMQ_PUB) {
  socket_.connect("ipc:///tmp/mission_command_stream.ipc");
}

void MissionMessageQueueSender::SendMission(
    ::lib::mission_message_queue::Mission mission_protobuf) {
  ::std::string serialized_mission_protobuf;
  mission_protobuf.SerializeToString(&serialized_mission_protobuf);

  ::zmq::message_t mission_protobuf_zmq(serialized_mission_protobuf.size());
  memcpy((void *)mission_protobuf_zmq.data(),
         serialized_mission_protobuf.c_str(),
         serialized_mission_protobuf.size());

  socket_.send(mission_protobuf_zmq);
}

// Receiver ////////////////////////////////////////////////////////////////////
MissionMessageQueueReceiver::MissionMessageQueueReceiver()
    : context_(1),
      socket_(context_, ZMQ_SUB),
      thread_(&MissionMessageQueueReceiver::ReceiveThread, this) {
}

MissionMessageQueueReceiver::~MissionMessageQueueReceiver() {
  Quit();
  thread_.join();
}


void MissionMessageQueueReceiver::ReceiveThread() {
  socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  socket_.bind("ipc:///tmp/mission_command_stream.ipc");

  // Listen to ZMQ message queue and update MissionManager with any new missions
  // that come along.
  ::zmq::message_t mission_message;

  while (run_) {
    if(!socket_.recv(&mission_message, ZMQ_NOBLOCK)) {
      usleep(1e6 / 1e4);
      continue;
    }

    ::std::string mission_message_string(
        static_cast<char *>(mission_message.data()), mission_message.size());

    ::lib::mission_message_queue::Mission mission_protobuf;
    mission_protobuf.ParseFromString(mission_message_string);

    mission_manager_.SetCommands(ParseMissionProtobuf(mission_protobuf));
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
