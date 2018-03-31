#ifndef LIB_MISSION_MESSAGE_QUEUE_MISSION_MESSAGE_QUEUE_H_
#define LIB_MISSION_MESSAGE_QUEUE_MISSION_MESSAGE_QUEUE_H_

#include <thread>

#include "zmq.hpp"

#include "lib/mission_manager/mission_manager.h"
#include "lib/mission_message_queue/mission_commands.pb.h"

namespace lib {
namespace mission_message_queue {

class MissionMessageQueueSender {
 public:
  MissionMessageQueueSender();

  void operator()();
  void Quit() { run_ = false; }

  ::std::vector<::std::shared_ptr<::lib::MissionCommand>> ParseMissionProtobuf(
      ::lib::mission_message_queue::Mission mission_protobuf);

 private:
  ::std::atomic<bool> run_{true};
};

class MissionMessageQueueReceiver {
 public:
  MissionMessageQueueReceiver(::lib::MissionManager *mission_manager);

  void operator()();
  void Quit() { run_ = false; }

  ::std::vector<::std::shared_ptr<::lib::MissionCommand>> ParseMissionProtobuf(
      ::lib::mission_message_queue::Mission mission_protobuf);

 private:
  ::lib::MissionManager *mission_manager_;

  ::std::atomic<bool> run_{true};
};

}  // mission_message_queue
}  // namespace lib

#endif  // LIB_MISSION_MESSAGE_QUEUE_MISSION_MESSAGE_QUEUE_H_
