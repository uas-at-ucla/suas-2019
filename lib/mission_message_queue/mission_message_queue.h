#ifndef LIB_MISSION_MESSAGE_QUEUE_MISSION_MESSAGE_QUEUE_H_
#define LIB_MISSION_MESSAGE_QUEUE_MISSION_MESSAGE_QUEUE_H_

#include <unistd.h>
#include <thread>
#include <atomic>
#include <functional>

#include "zmq.hpp"

#include "lib/mission_manager/mission_manager.h"
#include "lib/mission_message_queue/mission_commands.pb.h"

namespace lib {
namespace mission_message_queue {

// Sender //////////////////////////////////////////////////////////////////////
class MissionMessageQueueSender {
 public:
  MissionMessageQueueSender();

  void SendMission(::lib::mission_message_queue::Mission mission_protobuf);

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

  void Quit() { run_ = false; }

  ::lib::MissionManager *get_mission_manager() { return &mission_manager_; }

  ::std::vector<::std::shared_ptr<::lib::MissionCommand>> ParseMissionProtobuf(
      ::lib::mission_message_queue::Mission mission_protobuf);

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
