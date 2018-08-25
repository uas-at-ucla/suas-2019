#pragma once

#include <atomic>
#include <bitset>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "sio_client.h"
#include "sio_socket.h"
#include "zmq.hpp"

#include "lib/logger/log_sender.h"
#include "lib/serial_comms/serial_comms_bridge.h"
#include "lib/phased_loop/phased_loop.h"
#include "src/control/io/io.h"
#include "src/control/loops/flight_loop.h"
#include "src/control/messages.pb.h"

namespace src {
namespace control {
namespace ground_communicator {

class MissionReceiver {
 public:
  MissionReceiver();
  void Run();
  void RunIteration(int message_index);

  void OnConnect();
  void OnFail();

  void ConnectToGround();

  enum GoalState {
    STANDBY,
    RUN_MISSION,
    FAILSAFE,
    THROTTLE_CUT,
    TAKEOFF,
    HOLD,
    OFFBOARD,
    RTL,
    LAND,
    ARM,
    DISARM,
    ALARM,
    BOMB_DROP,
    DSLR
  };

 private:
  void SetState(::std::string new_state);
  MissionReceiver::GoalState GetState();
  void SetFlightLoopGoal(GoalState new_state);

  ::lib::mission_message_queue::MissionMessageQueueSender
      mission_message_queue_sender_;

  ::std::mutex state_mutex_;
  GoalState state_;
  ::sio::client client_;

  ::lib::phased_loop::PhasedLoop phased_loop_;

  ::std::atomic<bool> running_;

  double last_serial_telemetry_sent_;
  ::lib::serial_comms::SerialCommsBridge serial_comms_bridge_;

  void SendTelemetry(int message_index);
};

void on_connect();
void on_fail();

}  // namespace ground_communicator
}  // namespace control
}  // namespace src
