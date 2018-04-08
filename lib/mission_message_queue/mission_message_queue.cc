#include "mission_message_queue.h"

namespace lib {
namespace mission_message_queue {

// Sender //////////////////////////////////////////////////////////////////////
MissionMessageQueueSender::MissionMessageQueueSender()
    : context_(1), socket_(context_, ZMQ_PUB) {
  socket_.connect("ipc:///tmp/mission_command_stream.ipc");
}

void MissionMessageQueueSender::SendMission(
    ::lib::mission_manager::Mission mission_protobuf) {
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
      usleep(1e6 / 1e1);
      continue;
    }

    ::std::string mission_message_string(
        static_cast<char *>(mission_message.data()), mission_message.size());

    ::lib::mission_manager::Mission mission_protobuf;
    mission_protobuf.ParseFromString(mission_message_string);

    mission_manager_.SetCommands(mission_protobuf);
  }
}

}  // mission_message_queue
}  // namespace lib
