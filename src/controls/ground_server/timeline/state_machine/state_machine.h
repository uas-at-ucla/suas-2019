#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <limits>

namespace src {
namespace controls {
namespace ground_server {
namespace state_machine {

/**
 * Contains all special Result values
 */
namespace result {
  /**
   * A result is returned from the execution of one iteration of a state.
   * It can either be a negative number, in which case it is one of the special Result values
   * listed below, or it can be a the StateId of the next state to transition to
   */
  typedef int32_t Result;

  /**
   * Indicates that the State should continue executing
   */
  constexpr Result CONTINUE = -1;

  /**
   * Indicates that the State is finshed and should not be executed any more.
   * This should only be returned from a top-level StateMachine
   */ 
  constexpr Result FINISHED = -2;

  /**
   * Indicates that execution of the state machine should be paused until the next
   * time-slice of the execution loop
   */
  constexpr Result YIELD = -3;
}

/**
 * An identifier for a state which is part of a state machine
 */
typedef uint32_t StateId;

using result::Result;

/**
 * One state in a state machine.
 * The template parameter Context is the type of data which is passed to the Execute method.
 * This can be any global state which the Execute method requires.
 */
template<typename Context>
class State {
public:
  virtual result::Result Execute(Context ctx) = 0;
};

template<typename Context>
class SmartState: public State<Context> {
public:
  result::Result Execute(Context ctx) final override;

  virtual void Initialize(Context ctx);
  virtual result::Result Run(Context ctx) = 0;
  virtual void Finish(Context ctx);

private:
  bool has_initialized_;
  bool has_finished_;
};

constexpr StateId STATE_MACHINE_FINISHED = std::numeric_limits<StateId>::max();

template<typename Context>
class StateMachine: public State<Context> {
public:
  typedef std::shared_ptr<State<Context>> StatePtr;
  typedef std::map<StateId, StatePtr> States;

  StateMachine(States states, StateId initial_state_id);

  void Reset();

  result::Result Execute(Context ctx) override;

  bool IsFinished() const;

private:
  States states_;
  StateId initial_state_id_;
  StateId current_state_id_;
};

/**
 * An exception thrown when an invalid StateId is attempted to be executed.
 */
class InvalidStateException: std::exception {
public:
  InvalidStateException(result::Result result);

  const char* what() const noexcept override;

private:
  // result::Result result_;
  std::string what_;
};

template<typename Context>
Result SmartState<Context>::Execute(Context ctx) {
  if (this->has_finished_) {
    // TODO: should this be a problem?
    this->has_finished_ = this->has_initialized_ = false;
  }
  if (!this->has_initialized_) {
    this->Initialize(ctx);
    this->has_initialized_ = true;
  }
  Result res = this->Run(ctx);
  switch (res) {
    case result::CONTINUE:
    case result::YIELD:
      return res;
    default:
      this->Finish(ctx);
      this->has_finished_ = true;
      return res;
  }
}

template<typename Context>
void SmartState<Context>::Initialize(Context ctx) {
  (void) ctx;
}

template<typename Context>
void SmartState<Context>::Finish(Context ctx) {
  (void) ctx;
}

template<typename Context>
StateMachine<Context>::StateMachine(StateMachine::States states, StateId initial_state_id) :
  states_(states), initial_state_id_(initial_state_id) {
  if (this->states_.count(this->initial_state_id_) == 0) {
    throw InvalidStateException(this->initial_state_id_);
  }
  this->Reset();
}

template<typename Context>
void StateMachine<Context>::Reset() {
  this->current_state_id_ = this->initial_state_id_;
}

template<typename Context>
Result StateMachine<Context>::Execute(Context ctx) {
  typename States::iterator current_state = this->states_.find(this->current_state_id_);
  if (current_state == this->states_.end()) {
    // if the current state does not exist, there is some logic error
    // the exception should have been thrown at the state transition, so don't crash the program here
    return result::FINISHED;
  }
  Result res = current_state->second->Execute(ctx);
  switch (res) {
    case result::YIELD:
    case result::CONTINUE:
    case result::FINISHED:
      return res;
    default:
      if (res >= 0) {
        this->current_state_id_ = res;
        return result::CONTINUE;
      }
  }
  throw InvalidStateException(res);
}

template<typename Context>
bool StateMachine<Context>::IsFinished() const {
  return this->current_state_id_ == STATE_MACHINE_FINISHED;
}

}
}
}
}
