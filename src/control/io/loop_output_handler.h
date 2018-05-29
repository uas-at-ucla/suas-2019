#ifndef SPINNY_CONTROL_IO_LOOP_OUTPUT_HANDLER_H_
#define SPINNY_CONTROL_IO_LOOP_OUTPUT_HANDLER_H_

#include <chrono>

#include <sys/timerfd.h>
#include <atomic>

#include "aos/common/scoped_fd.h"
#include "aos/common/scoped_fd.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

namespace src {
namespace control {
namespace io {

class LoopOutputHandler {
 public:
  LoopOutputHandler(
      ::std::chrono::nanoseconds timeout = ::std::chrono::milliseconds(500));

  void Quit() { run_ = false; }

  void operator()();

 protected:
  // Read a message from the appropriate queue.
  // This must block.
  virtual void Read() = 0;

  // Send the output from the appropriate queue to the hardware.
  // Read() will always be called at least once before per invocation of this.
  virtual void Write() = 0;

  // Stop all outputs. This will be called in a separate thread (if at all).
  // The subclass implementation should handle any appropriate logging.
  // This will be called exactly once if Read() blocks for too long and once
  // after Quit is called.
  virtual void Stop() = 0;

 private:
  // The thread that actually contains a timerfd to implement timeouts. The
  // idea is to have a timerfd that is repeatedly set to the timeout expiration
  // in the future so it will never actually expire unless it is not reset for
  // too long.
  //
  // This class nicely encapsulates the raw fd and manipulating it. Its
  // operator() loops until Quit() is called, calling Stop() on its associated
  // LoopOutputHandler whenever the timerfd expires.
  class Watchdog {
   public:
    Watchdog(LoopOutputHandler *handler, ::std::chrono::nanoseconds timeout);

    void operator()();

    void Reset();

    void Quit() { run_ = false; }

   private:
    LoopOutputHandler *const handler_;

    const ::std::chrono::nanoseconds timeout_;

    ::aos::ScopedFD timerfd_;

    ::std::atomic<bool> run_{true};
  };

  ::aos::time::PhasedLoop phased_loop_;

  Watchdog watchdog_;

  ::std::atomic<bool> run_{true};
};

}  // namespace io
}  // namespace control
}  // namespace src

#endif  // SPINNY_CONTROL_IO_LOOP_OUTPUT_HANDLER_H_
