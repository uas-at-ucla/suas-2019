#include "mavlink_udp.h"

namespace lib {
namespace mavlink_udp {

MavlinkUDP::MavlinkUDP(int port)
    : context_(1), socket_(context_, ZMQ_STREAM) {
  zmq_bind(socket_, "tcp://*:8083");
}

int MavlinkUDP::read_messages(::std::vector<mavlink_message_t> &messages) {
  mavlink_status_t status;
  uint8_t msgReceived = false;

  // this function locks the port during read
  pthread_mutex_lock(&lock);

  ::zmq::message_t received_message;

  socket_.recv(&received_message);
  ::std::string received_string(
      static_cast<char *>(received_message.data()), received_message.size());

  pthread_mutex_unlock(&lock);

  ::std::cout << received_string << ::std::endl;
  for (size_t i = 0; i < received_string.size(); i++) {
    // the parsing
    mavlink_message_t message;

    msgReceived = mavlink_parse_char(MAVLINK_COMM_1, received_string.at(i), &message, &status);

    lastStatus = status;

    if (msgReceived) {
      ::std::cout << "MESSAGEE\n";
      messages.push_back(message);
    }
  }

  // Couldn't read from port
  if (received_string.size() < 1) {
    fprintf(stderr, "ERROR: Could not read from fd.\n");
    usleep(1e6 / 3);
  }

  // Done!
  return msgReceived;
}

int MavlinkUDP::write_message(const mavlink_message_t &message) {
  char buf[300];

  // Translate message to buffer
  unsigned len = mavlink_msg_to_send_buffer((uint8_t *)buf, &message);

  ::zmq::message_t msg(len);
  memcpy((void *)msg.data(), buf, len);

  // Send message.
  int write_success = socket_.send(msg);

  return write_success;
}

}  // namespace mavlink_udp
}  // namespace lib
