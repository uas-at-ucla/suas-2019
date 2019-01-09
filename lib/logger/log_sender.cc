#include "log_sender.h"

namespace lib {
namespace logger {

LogSender::LogSender() : context_(1), socket_(context_, ZMQ_PUB) {
  socket_.connect("ipc:///tmp/logger.ipc");
}

void LogSender::Log(const char *file, const char *function, int line_num,
                    ::std::ostringstream &log_line) {
  double time = ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
                    ::std::chrono::system_clock::now().time_since_epoch())
                    .count() /
                1e9;

  ::lib::logger::LogMessage log_message_proto;
  log_message_proto.set_line_number(line_num);
  log_message_proto.set_file(file);
  log_message_proto.set_function(function);
  log_message_proto.set_time(time);

  log_message_proto.mutable_log_line()->set_line(log_line.str());

  ::std::string serialized_log_message;
  log_message_proto.SerializeToString(&serialized_log_message);

  ::zmq::message_t log_message_zmq(serialized_log_message.size());
  memcpy((void *)log_message_zmq.data(), serialized_log_message.c_str(),
         serialized_log_message.size());

  socket_.send(log_message_zmq);
}

} // namespace logger
} // namespace lib
