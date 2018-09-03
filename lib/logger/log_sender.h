#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

#include "zmq.hpp"

#include "lib/logger/log_message.pb.h"

namespace lib {
namespace logger {

class LogSender {
 public:
  LogSender();
  ~LogSender();
  void Log(const char *file, const char *function, int line_num,
           ::std::ostringstream &log_line);

 private:
  ::zmq::context_t context_;
  ::zmq::socket_t socket_;
};

static LogSender *log_sender = new LogSender();

} // namespace logger
} // namespace lib

#define LOG_LINE(args)                                                         \
  do {                                                                         \
    ::std::ostringstream __log_line;                                           \
    __log_line << args;                                                        \
    ::lib::logger::log_sender->Log(__FILE__, __func__, __LINE__, __log_line);  \
  } while (0)
