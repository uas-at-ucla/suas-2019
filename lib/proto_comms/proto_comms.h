#pragma once

#include <atomic>
#include <functional>
#include <queue>
#include <thread>
#include <iostream>
#include <unistd.h>

#include "zmq.hpp"

#include "lib/proto_comms/proto_comms.h"

namespace lib {
namespace proto_comms {

class ProtoSender {
 public:
  ProtoSender(const char *bind_address);

  void Send(::std::string message);

 private:
  ::zmq::context_t context_;
  ::zmq::socket_t socket_;
};

class ProtoReceiver {
 public:
  ProtoReceiver(const char *bind_address, size_t max_size);
  ~ProtoReceiver();

  void Quit() { run_ = false; }

  ::std::string GetLatest();
  ::std::queue<::std::string> GetQueue();

 private:
  void ReceiveThread();
  
  ::std::atomic<bool> run_{true};

  ::zmq::context_t context_;
  ::zmq::socket_t socket_;

  ::std::thread thread_;

  size_t max_size_;
  ::std::queue<::std::string> received_messages_;
};

} // namespace proto_comms
} // namespace lib
