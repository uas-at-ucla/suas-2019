#include "state_machine.hh"

#include <sstream>

namespace src {
namespace controls {
namespace ground_server {
namespace state_machine {

InvalidStateException::InvalidStateException(Result result) {
  std::ostringstream what_stream;
  what_stream << "An attempt to execute an invalid state id was made: "
              << result;
  this->what_ = what_stream.str();
}

const char *InvalidStateException::what() const noexcept {
  return this->what_.c_str();
}

} // namespace state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
