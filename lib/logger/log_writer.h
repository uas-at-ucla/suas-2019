#pragma once

#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <functional>
#include <iomanip>
#include <string>
#include <sstream>
#include <thread>

#include "zmq.hpp"
#include "spdlog/spdlog.h"

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
  void ReplaceString(std::string& subject, const std::string& search,
                     const std::string& replace);

  ::std::atomic<bool> run_{true};

  ::zmq::context_t context_;
  ::zmq::socket_t socket_;

  ::std::thread thread_;

  ::std::shared_ptr<::spdlog::logger> logger_;
};

}  // namespace logger
}  // namespace lib

