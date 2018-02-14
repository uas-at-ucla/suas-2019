#include "loop_input_handler.h"

namespace spinny {
namespace control {
namespace io {

LoopInputHandler::LoopInputHandler()
    : phased_loop_(::std::chrono::milliseconds(5),
                   ::std::chrono::milliseconds(0)) {}

void LoopInputHandler::operator()() {
  while (run_) {
    RunIteration();
    phased_loop_.SleepUntilNext();
  }
}

}  // namespace io
}  // namespace control
}  // namespace spinny
