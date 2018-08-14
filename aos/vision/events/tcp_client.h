#pragma once

#include "aos/vision/events/epoll_events.h"

#include <memory>
#include <string>

namespace aos {
namespace events {

// Handles the client connection logic to hostname:portno
class TcpClient : public EpollEvent {
 public:
  TcpClient(const std::string &hostname, int portno);

  // Implement ReadEvent from EpollEvent to use this class.
};

}  // namespace events
}  // namespace aos

