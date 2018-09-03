#pragma once

#include <atomic>
#include <functional>
#include <thread>
#include <unistd.h>

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

} // namespace serial_comms
} // namespace lib
