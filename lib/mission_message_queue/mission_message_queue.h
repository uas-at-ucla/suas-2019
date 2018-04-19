#ifndef LIB_MISSION_MESSAGE_QUEUE_MISSION_MESSAGE_QUEUE_H_
#define LIB_MISSION_MESSAGE_QUEUE_MISSION_MESSAGE_QUEUE_H_

#include <unistd.h>
#include <atomic>
#include <functional>
#include <thread>

#include "zmq.hpp"

#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/mission_manager/mission_manager.h"

namespace lib {
namespace mission_message_queue {

// Sender //////////////////////////////////////////////////////////////////////
class MissionMessageQueueSender {
 public:
  MissionMessageQueueSender();

  void SendMission(::lib::mission_manager::Mission mission_protobuf);

 private:
  ::zmq::context_t context_;
  ::zmq::socket_t socket_;
  ::std::atomic<bool> run_{true};
};

// Receiver ////////////////////////////////////////////////////////////////////
class MissionMessageQueueReceiver {
 public:
  MissionMessageQueueReceiver();
  ~MissionMessageQueueReceiver();

  void RunPreprocessor(Position3D position);
  void SetObstacles(::lib::mission_manager::Obstacles obstacles);

  void Quit() { run_ = false; }

  ::lib::MissionManager *get_mission_manager() { return &mission_manager_; }
  void SetMission(::lib::mission_manager::Mission mission);

 private:
  void ReceiveThread();

  ::lib::MissionManager mission_manager_;

  ::std::atomic<bool> run_{true};

  ::zmq::context_t context_;
  ::zmq::socket_t socket_;

  ::std::thread thread_;
};

}  // mission_message_queue
}  // namespace lib

#endif  // LIB_MISSION_MESSAGE_QUEUE_MISSION_MESSAGE_QUEUE_H_
