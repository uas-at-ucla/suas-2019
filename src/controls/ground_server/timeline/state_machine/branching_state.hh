#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include "state.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace state_machine {

typedef uint32_t BranchId;

template<typename Context>
class BranchingState : public State<Context> {
public:
  typedef std::map<StateId, StateId> BranchMap;

  BranchingState() = default;

  virtual const std::vector<BranchId> ListBranches() const = 0;
  StateId Branch(BranchId branch_id) const;
  void SetBranch(BranchId branch_id, StateId state_id);

private:
  BranchMap branches_;
};

class InvalidBranchException: std::exception {
public:
  InvalidBranchException(BranchId branch_id);

  const char* what() const noexcept override;

private:
  std::string what_;
};

template<typename Context>
StateId BranchingState<Context>::Branch(BranchId branch_id) const {
  auto branch_it = branches_.find(branch_id);
  if (branch_it == branches_.end()) {
    return result::FINISHED;
  }
  return branch_it->second;
}

template<typename Context>
void BranchingState<Context>::SetBranch(BranchId branch_id, StateId state_id) {
  branches_[branch_id] = state_id;
}

}
}
}
}
