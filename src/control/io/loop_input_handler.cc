#include "loop_input_handler.h"

namespace src {
namespace control {
namespace io {

LoopInputHandler::LoopInputHandler() : phased_loop_(200) {}

void LoopInputHandler::operator()() {
  while (run_) {
    RunIteration();
    phased_loop_.SleepUntilNext();
  }
}

} // namespace io
} // namespace control
} // namespace src
