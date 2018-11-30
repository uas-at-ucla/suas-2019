#include "mission_message_queue.h"

namespace lib {
namespace mission_message_queue {

// Sender //////////////////////////////////////////////////////////////////////
MissionMessageQueueSender::MissionMessageQueueSender() :
    context_(1),
    socket_(context_, ZMQ_PAIR) {
  socket_.connect("ipc:///tmp/mission_command_stream.ipc");
}

void MissionMessageQueueSender::SendData(
    ::lib::mission_manager::GroundData ground_data) {
  ::std::string serialized_ground_data;
  ground_data.SerializeToString(&serialized_ground_data);

  ::zmq::message_t ground_data_protobuf_zmq(serialized_ground_data.size());
  memcpy((void *)ground_data_protobuf_zmq.data(),
         serialized_ground_data.c_str(), serialized_ground_data.size());

  socket_.send(ground_data_protobuf_zmq);
}

::lib::mission_manager::Mission MissionMessageQueueSender::GetMission() {
  ::zmq::message_t message_zmq(11);
  memcpy((void *)message_zmq.data(), "get_mission", 11);
  socket_.send(message_zmq);

  ::zmq::message_t mission_message;
  ::lib::mission_manager::Mission mission_protobuf;

  if (!socket_.recv(&mission_message, ZMQ_NOBLOCK)) {
    usleep(1e6 / 1e2);
    if (!socket_.recv(&mission_message, ZMQ_NOBLOCK)) {
      return mission_protobuf;
    }
  }

  ::std::string mission_string(static_cast<char *>(mission_message.data()),
                               mission_message.size());
  mission_protobuf.ParseFromString(mission_string);
  return mission_protobuf;
}

// Receiver ////////////////////////////////////////////////////////////////////
MissionMessageQueueReceiver::MissionMessageQueueReceiver() :
    context_(1),
    socket_(context_, ZMQ_PAIR),
    thread_(&MissionMessageQueueReceiver::ReceiveThread, this) {}

MissionMessageQueueReceiver::~MissionMessageQueueReceiver() {
  Quit();
  thread_.join();
}

void MissionMessageQueueReceiver::ReceiveThread() {
  // socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  socket_.bind("ipc:///tmp/mission_command_stream.ipc");

  // Listen to ZMQ message queue and update MissionManager with any new missions
  // that come along.
  ::zmq::message_t mission_message;

  while (run_) {
    if (!socket_.recv(&mission_message, ZMQ_NOBLOCK)) {
      usleep(1e6 / 1e1);
      continue;
    }

    ::std::string ground_data_string(
        static_cast<char *>(mission_message.data()), mission_message.size());

    if (ground_data_string == "get_mission") {
      ::std::string serialized_mission;
      mission_manager_.GetMission().SerializeToString(&serialized_mission);

      ::zmq::message_t mission_protobuf_zmq(serialized_mission.size());
      memcpy((void *)mission_protobuf_zmq.data(), serialized_mission.c_str(),
             serialized_mission.size());

      socket_.send(mission_protobuf_zmq);
    } else {
      ::lib::mission_manager::GroundData ground_data_protobuf;
      ground_data_protobuf.ParseFromString(ground_data_string);

      if (ground_data_protobuf.has_mission()) {
        mission_manager_.SetCommands(ground_data_protobuf.mission());
      } else if (ground_data_protobuf.has_obstacles()) {
        mission_manager_.SetObstacles(ground_data_protobuf.obstacles());
      }
    }
  }
}

void MissionMessageQueueReceiver::SetMission(
    ::lib::mission_manager::Mission mission) {
  mission_manager_.SetCommands(mission);
}

void MissionMessageQueueReceiver::SetObstacles(
    ::lib::mission_manager::Obstacles obstacles) {
  mission_manager_.SetObstacles(obstacles);
}

void MissionMessageQueueReceiver::RunPreprocessor(Position3D position) {
  mission_manager_.Preprocess(position);
}

} // namespace mission_message_queue
} // namespace lib
