#include "src/controls/ground_server/timeline/drone2state_visitor/drone2state_visitor.hh"

using namespace src::controls::ground_server::timeline::drone2state_visitor;

int main() {
  Drone2StateVisitor visitor;
  timeline::DroneProgram program;
  program.add_commands()->mutable_nothing_command();
  program.add_commands()->mutable_sleep_command()->set_time(2.0);
  program.add_commands()->mutable_nothing_command();
  program.add_commands()->mutable_sleep_command()->set_time(2.0);
  program.add_commands()->mutable_nothing_command();
  program.add_commands()->mutable_sleep_command()->set_time(2.0);
  program.add_commands()->mutable_nothing_command();
  DroneContext ctx;
  auto state_machine = visitor.Process(program);
  while (!state_machine.IsFinished()) {
    state_machine.Execute(ctx);
  }
}