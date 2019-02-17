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

#include "lib/base64_tools/base64_tools.h"
#include "lib/logger/log_sender.h"
#include "lib/phased_loop/phased_loop.h"
#include "lib/proto_comms/proto_comms.h"
#include "lib/serial_comms/serial_comms_bridge.h"
#include "src/controls/loops/flight_loop.h"
#include "src/controls/messages.pb.h"

namespace src {
namespace controls {
namespace ground_communicator {

void on_connect();
void on_fail();

class GroundCommunicator {
 public:
  GroundCommunicator();
  void Run();
  void RunIteration();

  void OnConnect();
  void OnFail();

  void ConnectToGround();

  enum GoalState {
    INIT,
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
  void SetGoal(GoalState new_state);

  ::lib::mission_message_queue::MissionMessageQueueSender
      mission_message_queue_sender_;

  GoalState state_;
  ::sio::client client_;

  ::src::controls::Goal goal_;

  ::lib::phased_loop::PhasedLoop phased_loop_;

  ::std::atomic<bool> running_;

  double last_serial_telemetry_sent_;
  ::lib::serial_comms::SerialCommsBridge serial_comms_bridge_;

  ::lib::proto_comms::ProtoReceiver<::src::controls::UasMessage>
      sensors_receiver_;
  ::lib::proto_comms::ProtoReceiver<::src::controls::Goal> goal_receiver_;
  ::lib::proto_comms::ProtoReceiver<::src::controls::Output> output_receiver_;

  ::lib::proto_comms::ProtoSender<::src::controls::Goal> goal_sender_;
};

} // namespace ground_communicator
} // namespace controls
} // namespace src
