#include <sstream>
#include <thread>
#include <unistd.h>

#include "gtest/gtest.h"

#include "lib/logger/log_sender.h"
#include "lib/logger/log_writer.h"

namespace lib {
namespace logger {
namespace testing {

TEST(LoggerTest, LogTest) {
  ::lib::logger::LogWriter log_writer;

  usleep(1e6);

  for (int i = 0; i < 1e3; i++) {
    LOG_LINE("test" << 1);
    LOG_LINE("test,hi" << 1);
  }

  usleep(1e4);
}

} // namespace testing
} // namespace logger
} // namespace lib
