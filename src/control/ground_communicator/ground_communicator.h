#ifndef SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_
#define SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_

#include <iostream>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <map>
#include <iomanip>

#include "sio_socket.h"
#include "sio_client.h"
#include "zmq.hpp"

#include "src/control/loops/flight_loop.q.h"
#include "src/control/loops/flight_loop.h"
#include "lib/mission_manager/mission_commands.pb.h"

namespace src {
namespace control {
namespace ground_communicator {

class MissionReceiver {
 public:
  MissionReceiver();
  void Run();
  void RunIteration();

  void OnConnect();
  void OnFail();

  void ConnectToGround();

  enum GoalState {
    RUN_MISSION,
    LAND,
    FAILSAFE,
    THROTTLE_CUT
  };

 private:
  void SetState(::std::string new_state);
  MissionReceiver::GoalState GetState();

  ::lib::mission_message_queue::MissionMessageQueueSender
      mission_message_queue_sender_;

  ::std::mutex state_mutex_;
  GoalState state_;
  ::sio::client client_;

  ::aos::time::PhasedLoop phased_loop_;

  ::std::atomic<bool> running_;

  int count_;

  void SendTelemetryPeriodic();
  void SendTelemetry();
};

void on_connect();
void on_fail();

}  // namespace ground_communicator
}  // namespace control
}  // namespace src

#endif  // SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_
