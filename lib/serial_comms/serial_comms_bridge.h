#ifndef LIB_SERIAL_COMMS_SERIAL_COMMS_BRIDGE_H_
#define LIB_SERIAL_COMMS_SERIAL_COMMS_BRIDGE_H_

#include <unistd.h>
#include <atomic>
#include <functional>
#include <thread>

#include "zmq.hpp"

#include "lib/serial_comms/serial_comms_message.pb.h"

namespace lib {
namespace serial_comms {

class SerialCommsBridge {
 public:
  SerialCommsBridge();

  void SendData(::lib::serial_comms::SerialCommsMessage message);

 private:
  ::zmq::context_t context_;
  ::zmq::socket_t socket_;
};

}  // namespace serial_comms
}  // namespace lib

#endif  // LIB_SERIAL_COMMS_SERIAL_COMMS_BRIDGE_H_
