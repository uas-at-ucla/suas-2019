#ifndef LIB_SERIAL_SERIAL_PORT_H_
#define LIB_SERIAL_SERIAL_PORT_H_

#include <iostream>
#include <string>
#include <vector>
#include <unistd.h> 

#include "zmq.hpp"
#include <common/mavlink.h>

namespace lib {
namespace mavlink_udp {

class MavlinkUDP {
 public:
  MavlinkUDP(int port);
  ~MavlinkUDP();

  int read_messages(::std::vector<mavlink_message_t> &message);
  int write_message(const mavlink_message_t &message);

 private:
  ::zmq::context_t context_;
  ::zmq::socket_t socket_;

  mavlink_status_t lastStatus;
  pthread_mutex_t lock;
};

}  // namespace mavlink_udp
}  // namespace lib

#endif  // LIB_SERIAL_SERIAL_PORT_H_
