#include "mavlink_udp.h"

namespace lib {
namespace mavlink_udp {

bool resolve_address_udp(::boost::asio::io_service &io, size_t chan,
                         ::std::string host, unsigned short port,
                         ::boost::asio::ip::udp::endpoint &ep) {
  bool result = false;
  ::boost::asio::ip::udp::resolver resolver(io);
  ::boost::system::error_code ec;

  ::boost::asio::ip::udp::resolver::query query(host, "");

  auto fn = [&](const ::boost::asio::ip::udp::endpoint &q_ep) {
    ep = q_ep;
    ep.port(port);
    result = true;
  };

  for (auto q_ep : resolver.resolve(query, ec)) fn(q_ep);

  if (ec) {
    fprintf(stderr, "ERROR: resolve error: %s\n", ec.message().c_str());
    result = false;
  }

  return result;
}

MavlinkUDP::MavlinkUDP() : socket_(io_service_) {
}

int MavlinkUDP::read_messages(::std::vector<mavlink_message_t> &messages) {
  mavlink_status_t status;
  uint8_t msgReceived = false;

  // this function locks the port during read
//pthread_mutex_lock(&lock);

  auto sthis = shared_from_this();
  socket_.async_receive_from(
      buffer(rx_buf), remote_ep,
      [sthis](::std::error_code error, size_t bytes_transferred) {
        if (error) {
          fprintf(stderr, "ERROR: receive error: %s\n", sthis->conn_id, error.message().c_str());
          sthis->close();
          return;
        }

        if (sthis->remote_ep != sthis->last_remote_ep) {
          sthis->remote_exists_ = true;
          sthis->last_remote_ep_ = sthis->remote_ep;
        }

        sthis->parse_buffer(PFX, sthis->rx_buf.data(), sthis->rx_buf.size(),
                            bytes_transferred);
        sthis->do_recvfrom();
      });

//pthread_mutex_unlock(&lock);

  ::std::cout << received_string << ::std::endl;
  for (size_t i = 0; i < received_string.size(); i++) {
    // the parsing
    mavlink_message_t message;

    msgReceived = mavlink_parse_char(MAVLINK_COMM_1, received_string.at(i),
                                     &message, &status);

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

  // Send message.
  //int write_success = socket_.send(msg);

  return 0;//write_success;
}

void MavlinkUDP::Close() {
  socket_.cancel();
  socket_.close();
}

}  // namespace mavlink_udp
}  // namespace lib
