#pragma once

#include <atomic>
#include <functional>
#include <thread>
#include <unistd.h>

#include "zmq.hpp"

#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/mission_manager/mission_manager.h"

namespace lib {
namespace mission_message_queue {

// Sender //////////////////////////////////////////////////////////////////////
class MissionMessageQueueSender {
 public:
  MissionMessageQueueSender();

  void SendData(::lib::mission_manager::GroundData ground_data);
  ::lib::mission_manager::Mission GetMission();

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

} // namespace mission_message_queue
} // namespace lib
