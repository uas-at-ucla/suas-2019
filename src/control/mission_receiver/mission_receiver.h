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
  void OnConnect();
  void OnFail();

 private:
  ::sio::client client_;
};

void on_connect();
void on_fail();

}  // namespace mission_receiver
}  // namespace control
}  // namespace spinny

#endif  // SPINNY_CONTROL_MISSION_RECEIVER_MISSION_RECEIVER_H_
