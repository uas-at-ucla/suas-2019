#pragma once

#include <cstdint>

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
 * It can either be a negative number, in which case it is one of the special
 * Result values listed below, or it can be a the StateId of the next state to
 * transition to
 */
typedef int32_t Result;

/**
 * Indicates that execution of the state machine should be paused until the next
 * time-slice of the execution loop, when execution of the current state will
 * resume.
 */
constexpr Result YIELD = -1;

/**
 * Indicates that the State is finshed and should not be executed any more.
 */
constexpr Result FINISHED = -2;
} // namespace result

/**
 * An identifier for a state which is part of a state machine
 */
typedef uint32_t StateId;

/**
 * One state in a state machine.
 * The template parameter Context is the type of data which is passed to the
 * Execute method. This can be any global state which the Execute method
 * requires.
 */
template <typename Context> class State {
 public:
  State();
  State(std::string name);

  virtual result::Result Execute(Context ctx);

  bool CurrentlyExecuting() const;
  const std::string &Name() const;

 protected:
  virtual void Initialize(Context ctx);
  virtual result::Result Step(Context ctx) = 0;
  virtual void Finish(Context ctx);

  std::string name_;

 private:
  bool has_initialized_;
  bool has_finished_;
};

////////////////////////////////////////
// Template implementations
////////////////////////////////////////

template <typename Context>
State<Context>::State() :
    State("State") {}

template <typename Context>
State<Context>::State(std::string name) :
    name_(name),
    has_initialized_(false),
    has_finished_(false) {}

template <typename Context>
result::Result State<Context>::Execute(Context ctx) {
  if (this->has_finished_) {
    // TODO: should this be a problem?
    this->has_finished_ = this->has_initialized_ = false;
  }
  if (!this->has_initialized_) {
    this->Initialize(ctx);
    this->has_initialized_ = true;
  }
  result::Result res = this->Step(ctx);
  switch (res) {
    case result::YIELD:
      return res;
    default:
      this->Finish(ctx);
      this->has_finished_ = true;
      return res;
  }
}

template <typename Context> bool State<Context>::CurrentlyExecuting() const {
  return this->has_initialized_ && !this->has_finished_;
}

template <typename Context> const std::string &State<Context>::Name() const {
  return this->name_;
}

template <typename Context> void State<Context>::Initialize(Context ctx) {
  (void)ctx;
}

template <typename Context> void State<Context>::Finish(Context ctx) {
  (void)ctx;
}

} // namespace state_machine
} // namespace ground_server
} // namespace controls
} // namespace src
