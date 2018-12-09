#pragma once

#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "state.hh"
#include "branching_state.hh"

namespace src {
namespace controls {
namespace ground_server {
namespace state_machine {

using result::Result;

constexpr StateId STATE_MACHINE_FINISHED = std::numeric_limits<StateId>::max();

template <typename Context> class StateMachine : public State<Context> {
 public:
  typedef std::shared_ptr<State<Context>> StatePtr;
  typedef std::map<StateId, StatePtr> States;

  StateMachine(States states, StateId initial_state_id);

  void Reset();

  result::Result Step(Context ctx) override;

  bool IsFinished() const;

 private:
  States states_;
  StateId initial_state_id_;
  typename States::iterator current_state_it_;
};

/**
 * An exception thrown when an invalid StateId is attempted to be executed.
 */
class InvalidStateException : std::exception {
 public:
  InvalidStateException(result::Result result);

  const char *what() const noexcept override;

 private:
  // result::Result result_;
  std::string what_;
};

template <typename Context>
StateMachine<Context>::StateMachine(StateMachine::States states,
                                    StateId initial_state_id) :
    states_(states),
    initial_state_id_(initial_state_id) {
  if (this->states_.count(this->initial_state_id_) == 0) {
    throw InvalidStateException(this->initial_state_id_);
  }
  this->Reset();
}

template <typename Context> void StateMachine<Context>::Reset() {
  current_state_it_ = states_.find(initial_state_id_);
}

template <typename Context> Result StateMachine<Context>::Step(Context ctx) {
  Result res;
  if (IsFinished()) {
    return result::FINISHED;
  }
  while (res >= 0) {
    res = current_state_it_->second->Execute(ctx);
    if (res >= 0) {
      auto new_current_it = states_.find(res);
      if (new_current_it == states_.end()) {
        throw InvalidStateException(res);
      }
      current_state_it_ = new_current_it;
    }
  }
  if (res == result::FINISHED) {
    current_state_it_ = states_.end();
  }
  return res;
}

template <typename Context> bool StateMachine<Context>::IsFinished() const {
  return current_state_it_ == states_.end();
}

} // namespace state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
