#include "loop_handler.h"

namespace src {
namespace control {
namespace io {

LoopHandler::LoopHandler() : phased_loop_(200) {}

void LoopHandler::operator()() {
  while (run_) {
    RunIteration();

    phased_loop_.SleepUntilNext();
  }

  Stop();
}

} // namespace io
} // namespace control
} // namespace src
