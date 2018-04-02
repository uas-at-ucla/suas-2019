#ifndef LIB_LOGGER_LOG_SENDER_H_
#define LIB_LOGGER_LOG_SENDER_H_

#include <unistd.h>
#include <atomic>
#include <functional>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>

#include "zmq.hpp"

#include "lib/logger/log_message.pb.h"

namespace lib {
namespace logger {

class LogSender {
 public:
  LogSender();
  void Log(::std::string file, ::std::string function, int line_num,
           ::std::ostringstream log_line);

 private:
  ::zmq::context_t context_;
  ::zmq::socket_t socket_;
};

static LogSender log_sender;

}  // namespace logger
}  // namespace lib

#define LOG_LINE(args)                                             \
  do {                                                             \
    ::lib::logger::log_sender.Log(__FILE__, __func__, __LINE__,    \
                                  ::std::ostringstream() << args); \
  } while (0)

#endif  // LIB_LOGGER_LOG_SENDER_H_
