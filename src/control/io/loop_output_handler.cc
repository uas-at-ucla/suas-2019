#include "loop_output_handler.h"

namespace src {
namespace control {
namespace io {

LoopOutputHandler::LoopOutputHandler(::std::chrono::nanoseconds timeout)
    : watchdog_(this, timeout) {}

void LoopOutputHandler::operator()() {
  ::std::thread watchdog_thread(::std::ref(watchdog_));

  while (run_) {
    Read();
    watchdog_.Reset();
    Write();
  }

  Stop();

  watchdog_.Quit();
  watchdog_thread.join();
}

LoopOutputHandler::Watchdog::Watchdog(LoopOutputHandler *handler,
                                      ::std::chrono::nanoseconds timeout)
    : handler_(handler),
      timeout_(timeout),
      timerfd_(timerfd_create(CLOCK_MONOTONIC, 0)) {
  if (timerfd_.get() == -1) {
    PLOG(FATAL, "timerfd_create(CLOCK_MONOTONIC, 0)");
  }
}

void LoopOutputHandler::Watchdog::operator()() {
  uint8_t buf[8];
  while (run_) {
    PCHECK(read(timerfd_.get(), buf, sizeof(buf)));
    handler_->Stop();
  }
}

void LoopOutputHandler::Watchdog::Reset() {
  itimerspec value = itimerspec();
  value.it_value.tv_sec =
      ::std::chrono::duration_cast<::std::chrono::seconds>(timeout_).count();
  value.it_value.tv_nsec =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          timeout_ - ::std::chrono::seconds(value.it_value.tv_sec))
          .count();
  PCHECK(timerfd_settime(timerfd_.get(), 0, &value, nullptr));
}

}  // namespace io
}  // namespace control
}  // namespace src
