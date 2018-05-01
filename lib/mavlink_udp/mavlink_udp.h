#ifndef LIB_SERIAL_SERIAL_PORT_H_
#define LIB_SERIAL_SERIAL_PORT_H_

#include <unistd.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <common/mavlink.h>
#include <boost/asio.hpp>

namespace lib {
namespace mavlink_udp {

class MavlinkUDP : public std::enable_shared_from_this<MavlinkUDP> {
 public:
  MavlinkUDP();
  ~MavlinkUDP();

  int read_messages(::std::vector<mavlink_message_t> &message);
  int write_message(const mavlink_message_t &message);

 private:
  mavlink_status_t lastStatus;
  pthread_mutex_t lock;

  ::boost::asio::io_service io_service_;
  std::atomic<bool> remote_exists_;
  ::boost::asio::ip::udp::socket socket_, remote_ep_, last_remote_ep_, bind_ep_;
};

}  // namespace mavlink_udp
}  // namespace lib

#endif  // LIB_SERIAL_SERIAL_PORT_H_
