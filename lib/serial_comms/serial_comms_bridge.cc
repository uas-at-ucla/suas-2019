#include "serial_comms_bridge.h"

namespace lib {
namespace serial_comms {

SerialCommsBridge::SerialCommsBridge() :
    context_(1),
    socket_(context_, ZMQ_PUB) {
  socket_.connect("ipc:///tmp/serial_comms.ipc");
}

void SerialCommsBridge::SendData(
    ::lib::serial_comms::SerialCommsMessage message) {

  ::std::string serialized_protobuf;
  message.SerializeToString(&serialized_protobuf);

  ::zmq::message_t zmq_protobuf(serialized_protobuf.size());
  memcpy((void *)zmq_protobuf.data(), serialized_protobuf.c_str(),
         serialized_protobuf.size());

  socket_.send(zmq_protobuf);
}

} // namespace serial_comms
} // namespace lib
