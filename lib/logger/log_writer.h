#pragma once

#include <algorithm>
#include <atomic>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

#include "spdlog/spdlog.h"
#include "zmq.hpp"

#include "lib/logger/log_message.pb.h"

namespace lib {
namespace logger {

class LogWriter {
 public:
  LogWriter();
  ~LogWriter();

  void Quit() { run_ = false; }

 private:
  void ReceiveThread();
  void ReplaceString(std::string &subject, const std::string &search,
                     const std::string &replace);

  ::std::atomic<bool> run_{true};

  ::zmq::context_t context_;
  ::zmq::socket_t socket_;

  ::std::thread thread_;

  ::std::shared_ptr<::spdlog::logger> logger_;
};

} // namespace logger
} // namespace lib
