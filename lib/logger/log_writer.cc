#include "log_writer.h"

namespace lib {
namespace logger {

#ifdef UAS_AT_UCLA_DEPLOYMENT
const char* kLogFileLocation = "/home/pi/logs/uas_at_ucla/drone_code.csv";
#else
const char* kLogFileLocation = "/tmp/drone_code.csv";
#endif

LogWriter::LogWriter()
    : context_(1),
      socket_(context_, ZMQ_SUB),
      thread_(&LogWriter::ReceiveThread, this),
      logger_(::spdlog::rotating_logger_mt("suas2018_log", kLogFileLocation,
                                           1024 * 1024 * 50, 3)) {
  logger_->set_pattern("%v");
}

LogWriter::~LogWriter() {
  Quit();
  thread_.join();
}

void LogWriter::ReceiveThread() {
  socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  socket_.bind("ipc:///tmp/logger.ipc");

  // Listen to ZMQ message queue for incoming log messages.
  ::zmq::message_t log_message;

  auto last_flush = ::std::chrono::steady_clock::now();

  while (run_) {
    if (!socket_.recv(&log_message, ZMQ_NOBLOCK)) {
      usleep(1e6 / 1e4);
      continue;
    }

    ::std::string log_message_string(static_cast<char*>(log_message.data()),
                                     log_message.size());

    ::lib::logger::LogMessage log_message_proto;
    log_message_proto.ParseFromString(log_message_string);

    // Remove commas to avoid messing up our CSV design.
    ::std::string log_line_no_commas = log_message_proto.log_line().line();
    ReplaceString(log_line_no_commas, ",", "<comma>");

    ::std::ostringstream log_line_csv_sstream;
    log_line_csv_sstream << ::std::fixed << ::std::setprecision(9)
                         << ::std::setfill('0') << ::std::setw(25)
                         << log_message_proto.time() << ","
                         << log_message_proto.file() << ","
                         << log_message_proto.line_number() << ","
                         << log_message_proto.function() << ","
                         << log_line_no_commas;
    ::std::string log_line_csv = log_line_csv_sstream.str();
    logger_->info(log_line_csv);

    // Periodically flush log data to file.
    if (::std::chrono::duration_cast<::std::chrono::milliseconds>(
            ::std::chrono::steady_clock::now() - last_flush).count() > 1e1) {
      logger_->flush();
      last_flush = ::std::chrono::steady_clock::now();
    }
  }
}

void LogWriter::ReplaceString(std::string& subject, const std::string& search,
                              const std::string& replace) {
  size_t pos = 0;
  while ((pos = subject.find(search, pos)) != std::string::npos) {
    subject.replace(pos, search.length(), replace);
    pos += replace.length();
  }
}

}  // namespace logger
}  // namespace lib
