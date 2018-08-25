#include "loop_output_handler.h"

namespace src {
namespace control {
namespace io {

LoopOutputHandler::LoopOutputHandler() : phased_loop_(200) {}

void LoopOutputHandler::operator()() {
  while (run_) {
    Read();
    Write();

    phased_loop_.SleepUntilNext();
  }

  Stop();
}

}  // namespace io
}  // namespace control
}  // namespace src
