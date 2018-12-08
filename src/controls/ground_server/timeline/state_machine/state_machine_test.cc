#include "branching_state.hh"
#include "state_machine.hh"

#include <iostream>
#include <thread>

using std::make_shared;

namespace state_machine = src::controls::ground_server::state_machine;

class StateContext {
 public:
  std::ostream &output_stream;
};

typedef StateContext &Context;
typedef state_machine::State<Context> State;
typedef state_machine::BranchingState<Context> BranchingState;
typedef state_machine::StateMachine<Context> StateMachine;

class TestState : public BranchingState {
 public:
  constexpr static state_machine::BranchId FINISH = 1;

  int counter;
  std::string name;

  TestState(std::string name) : name(name) {}

  const std::vector<state_machine::BranchId> ListBranches() const override {
    return {FINISH};
  }

  void Initialize(Context ctx) {
    counter = 0;
    ctx.output_stream << "state " << name << " initializing" << std::endl;
  }

  state_machine::Result Step(Context ctx) {
    ctx.output_stream << "this is iteration " << counter++ << " of state "
                      << name << std::endl;
    return (counter > 10) ? Branch(FINISH) : state_machine::result::YIELD;
  }

  void Finish(Context ctx) {
    ctx.output_stream << "state " << name << " finished" << std::endl;
  }
};

int main() {
  StateMachine::States states;
  auto state0 = make_shared<TestState>("state 1");
  state0->SetBranch(TestState::FINISH, 1);
  states[0] = state0;
  states[1] = make_shared<TestState>("state 2");

  StateContext ctx{output_stream : std::cout};

  StateMachine state_machine_(states, 0);
  while (!state_machine_.IsFinished()) {
    state_machine_.Execute(ctx);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}