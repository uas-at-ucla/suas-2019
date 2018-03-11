#ifndef SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_
#define SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_

#include <iostream>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <map>

#include "sio_socket.h"
#include "sio_client.h"
#include "zmq.hpp"

#include "src/control/loops/flight_loop.q.h"
#include "src/control/loops/flight_loop.h"
#include "src/control/mission_receiver/mission_commands.pb.h"

namespace spinny {
namespace control {
namespace mission_receiver {

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

  ::std::mutex state_mutex_;
  GoalState state_;
  ::sio::client client_;

  ::zmq::context_t context_;
  ::zmq::socket_t mission_command_stream_;

  ::aos::time::PhasedLoop phased_loop_;

  ::std::atomic<bool> running_;

  int count_;

  void SendTelemetryPeriodic();
  void SendTelemetry();
};

void on_connect();
void on_fail();

}  // namespace mission_receiver
}  // namespace control
}  // namespace spinny

#endif  // SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_
