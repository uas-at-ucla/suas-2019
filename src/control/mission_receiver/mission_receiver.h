#ifndef SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_
#define SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_

#include "sio_socket.h"
#include "sio_client.h"

#include "src/control/loops/flight_loop.q.h"

namespace spinny {
namespace control {
namespace mission_receiver {

class MissionReceiver {
 public:
  MissionReceiver();
  void Run();
};

}  // namespace mission_receiver
}  // namespace control
}  // namespace spinny

#endif  // SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_
