#include "branching_state.hh"

#include <sstream>

namespace src {
namespace controls {
namespace ground_server {
namespace state_machine {

InvalidBranchException::InvalidBranchException(BranchId branch_id)
{
  std::ostringstream what_stream;
  what_stream << "An attempt to execute an invalid branch id was made: " << branch_id;
  this->what_ = what_stream.str();
}

const char* InvalidBranchException::what() const noexcept {
  return this->what_.c_str();
}

}
}
}
}

