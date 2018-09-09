#pragma once

#include <atomic>
#include <fstream>
#include <functional>
#include <iostream>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <type_traits>
#include <unistd.h>

#include "zmq.hpp"
#include <google/protobuf/message.h>

#include "lib/proto_comms/proto_comms.h"

namespace lib {
namespace proto_comms {

template <typename T> class ProtoSender {
 public:
  ProtoSender(const char *bind_address) : socket_(context_, ZMQ_PUB) {

    static_assert(::std::is_base_of<::google::protobuf::Message, T>::value,
                  "Template must be derived from a protobuf");

    if (access(bind_address + 6, F_OK) != -1) {
      socket_.connect(bind_address);
    } else {
      socket_.bind(bind_address);
    }
  }

  void Send(T proto_message) {
    ::std::string proto_string;
    proto_message.SerializeToString(&proto_string);

    ::zmq::message_t zmq_message(proto_string.size());
    memcpy((void *)zmq_message.data(), proto_string.c_str(),
           proto_string.size());
    socket_.send(zmq_message);
  }

 private:
  ::zmq::context_t context_;
  ::zmq::socket_t socket_;
};

template <class T> class ProtoReceiver {
 public:
  ProtoReceiver(const char *bind_address, size_t max_size)
      : socket_(context_, ZMQ_SUB),                   //
        thread_(&ProtoReceiver::ReceiveThread, this), //
        max_size_(max_size) {

    socket_.connect(bind_address);

    ::std::ifstream ipc_file(bind_address);
    socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    int linger = 0;
    socket_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
  }

  ~ProtoReceiver() {
    Quit();
    socket_.close();
    context_.~context_t();
    thread_.join();
  }

  void Quit() { run_ = false; }

  bool HasMessages() { return received_messages_.size() > 0; }

  T GetLatest() { return received_messages_.back(); }

  ::std::queue<T> GetQueue() { return received_messages_; }

 private:
  void ReceiveThread() {
    // Listen to ZMQ messages and add them to the received queue.
    ::zmq::message_t new_message;

    while (run_) {
      try {
        socket_.recv(&new_message);
        ::std::string new_message_str(static_cast<char *>(new_message.data()),
                                      new_message.size());
        T proto_message;
        proto_message.ParseFromString(new_message_str);
        received_messages_.push(proto_message);
      } catch (zmq::error_t &e) {
        break;
      }

      if (received_messages_.size() > max_size_) {
        received_messages_.pop();
      }
    }
  }

  ::std::atomic<bool> run_{true};

  ::zmq::context_t context_;
  ::zmq::socket_t socket_;

  ::std::thread thread_;

  size_t max_size_;
  ::std::queue<T> received_messages_;
};

} // namespace proto_comms
} // namespace lib
