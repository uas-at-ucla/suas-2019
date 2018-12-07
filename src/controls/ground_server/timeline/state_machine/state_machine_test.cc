#include "state_machine.h"

#include <iostream>
#include <thread>

using std::make_shared;

namespace state_machine = src::controls::ground_server::state_machine;

typedef std::nullptr_t Context;
typedef state_machine::State<Context> State;
typedef state_machine::SmartState<Context> SmartState;
typedef state_machine::StateMachine<Context> StateMachine;

class TestState: public SmartState {
public:
  int counter;
  std::string name;

  TestState(std::string name) : name(name) {
  }

  void Initialize(Context ctx) {
    (void) ctx;
    counter = 0;
    std::cout << "state " << name << " initializing" << std::endl;
  }

  state_machine::Result Run(Context ctx) {
    std::cout << "this is iteration " << counter++ << " of state " << name << std::endl;
    (void) ctx;
    return (counter > 10) ? state_machine::result::FINISHED : state_machine::result::YIELD;
  }

  void Finish(Context ctx) {
    (void) ctx;
    std::cout << "state " << name << " finished" << std::endl;
  }
};

int main() {
  StateMachine::States states;
  states[0] = make_shared<TestState>("state 1");
  states[1] = make_shared<TestState>("state 2");

  StateMachine state_machine_(states, 0);
  while (!state_machine_.IsFinished()) {
    state_machine_.Execute(nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}