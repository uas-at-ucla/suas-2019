#include "proto_comms.h"

namespace lib {
namespace proto_comms {

// Sender //////////////////////////////////////////////////////////////////////
ProtoSender::ProtoSender(const char *bind_address)
    : socket_(context_, ZMQ_PUB) {
  socket_.connect(bind_address);
}

void ProtoSender::Send(::zmq::message_t *message) {
  socket_.send(*message);
}

// Receiver ////////////////////////////////////////////////////////////////////
ProtoReceiver::ProtoReceiver(const char *bind_address, size_t max_size)
    : socket_(context_, ZMQ_SUB),
      thread_(&ProtoReceiver::ReceiveThread, this), max_size_(max_size) {
  socket_.bind(bind_address);
  int linger = 0;
  socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  socket_.setsockopt(ZMQ_LINGER, &linger, sizeof (linger));
}

ProtoReceiver::~ProtoReceiver() {
  Quit();
  socket_.close();
  context_.~context_t();
  thread_.join();
}

void ProtoReceiver::ReceiveThread() {
  // Listen to ZMQ messages and add them to the received queue.
  ::zmq::message_t new_message;

  while (run_) {
    try {
      socket_.recv(&new_message);
      ::std::string new_message_str(
          static_cast<char *>(new_message.data()), new_message.size());

      ::std::cout << "GOT " << new_message_str << ::std::endl;

      received_messages_.push(new_message_str);
    } catch(zmq::error_t& e) {
      break;
    }

    if(received_messages_.size() > max_size_) {
      received_messages_.pop();
    }
  }

  ::std::cout << "JOIN!\n";
}

::std::string ProtoReceiver::GetLatest() {
  if(received_messages_.size() > 0) {
    return received_messages_.back();
  } else {
    return ::std::string();
  }
}

::std::queue<::std::string> ProtoReceiver::GetQueue() {
  return received_messages_;
}

} // namespace proto_comms
} // namespace lib
