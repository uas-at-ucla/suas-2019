#include "branching_state.hh"
#include "state_machine.hh"

#include <gtest/gtest.h>

#include <iostream>
#include <thread>

using std::make_shared;

namespace state_machine = src::controls::ground_server::state_machine;

struct StateContext {
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
  int max_count = 10;

  TestState(std::string name) : BranchingState() { this->name_ = name; }

  const std::vector<state_machine::BranchId> ListBranches() const override {
    return {FINISH};
  }

  void Initialize(Context ctx) override {
    counter = 0;
    ctx.output_stream << "state " << Name() << " initializing" << std::endl;
  }

  state_machine::Result Step(Context ctx) override {
    ctx.output_stream << "this is iteration " << counter++ << " of state "
                      << Name() << std::endl;
    return (counter >= max_count) ? Branch(FINISH)
                                  : state_machine::result::YIELD;
  }

  void Finish(Context ctx) override {
    ctx.output_stream << "state " << Name() << " finished" << std::endl;
  }
};

TEST(StateMachineTest, StateMachine) {
  StateMachine::States states;
  auto state0 = make_shared<TestState>("state 1");
  state0->SetBranch(TestState::FINISH, 1);
  states[0] = state0;
  auto state1 = make_shared<TestState>("state 2");
  state1->max_count = 20;
  states[1] = state1;

  StateContext ctx = {.output_stream = std::cout};

  EXPECT_EQ(state0->CurrentlyExecuting(), false);
  EXPECT_EQ(state1->CurrentlyExecuting(), false);

  StateMachine state_machine_(states, 0);
  int iterations = 0;
  while (!state_machine_.IsFinished()) {
    if (iterations == 5) {
      EXPECT_EQ(state0->CurrentlyExecuting(), true);
      EXPECT_EQ(state1->CurrentlyExecuting(), false);
    }
    if (iterations == 20) {
      EXPECT_EQ(state0->CurrentlyExecuting(), false);
      EXPECT_EQ(state1->CurrentlyExecuting(), true);
    }
    state_machine_.Execute(ctx);
    iterations++;
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(iterations, 29); // 9 state0, 1 state0+state1, 18 state1

  EXPECT_EQ(state0->CurrentlyExecuting(), false);
  EXPECT_EQ(state1->CurrentlyExecuting(), false);

  EXPECT_EQ(state0->counter, 10);
  EXPECT_EQ(state1->counter, 20);
}
