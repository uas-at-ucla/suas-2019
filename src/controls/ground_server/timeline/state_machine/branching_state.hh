#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include "state.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace state_machine {

/**
 * The identifier of a branch for a BranchingState
 */
typedef uint32_t BranchId;

/**
 * A state which can branch to other states. Each of those branches can be
 * set up to result in a different StateId.
 * Subclasses should have many `constexpr static BranchId` members with the
 * different BranchIds which are possible to be set.
 */
template <typename Context> class BranchingState : public State<Context> {
 public:
  typedef std::map<StateId, StateId> BranchMap;

  BranchingState() = default;
  BranchingState(std::string name);

  /**
   * Lists all possible branch ids which can be set in SetBranch
   */
  virtual const std::vector<BranchId> ListBranches() const = 0;

  /**
   * Gets the resultant StateId for the BranchId. Primarily used in
   * implementations of subclasses
   * If a StateId has not been set, it will default to FINISHED and terminate
   * the state machine.
   */
  StateId Branch(BranchId branch_id) const;

  /**
   * Sets what state_id the state will attempt to execute for the specified
   * branch_id
   */
  void SetBranch(BranchId branch_id, StateId state_id);

 private:
  BranchMap branches_;
};

template <typename Context>
BranchingState<Context>::BranchingState(std::string name) :
    State<Context>(name) {}

template <typename Context>
StateId BranchingState<Context>::Branch(BranchId branch_id) const {
  auto branch_it = branches_.find(branch_id);
  if (branch_it == branches_.end()) {
    return result::FINISHED;
  }
  return branch_it->second;
}

template <typename Context>
void BranchingState<Context>::SetBranch(BranchId branch_id, StateId state_id) {
  branches_[branch_id] = state_id;
}

} // namespace state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
