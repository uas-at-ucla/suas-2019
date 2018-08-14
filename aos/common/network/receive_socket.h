#pragma once

#include "aos/common/network/socket.h"

namespace aos {
namespace network {

class ReceiveSocket : public Socket {
 public:
  explicit ReceiveSocket(NetworkPort port) { Connect(port); }
  int Connect(NetworkPort port);
};

}  // namespace network
}  // namespace aos

