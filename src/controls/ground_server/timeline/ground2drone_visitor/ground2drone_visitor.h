#pragma once

#include "lib/mission_manager/mission_commands.pb.h"
#include "src/controls/ground_server/timeline/timeline_grammar.pb.h"

// using protobuf messages defined in this namespace
using namespace lib::mission_manager;

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace ground2drone_visitor {

class Ground2DroneVisitor {
 public:
  Ground2DroneVisitor();
  bool Process(GroundProgram *input_program, DroneProgram& drone_program);

 private:
  void ConcatenateDroneProgramCommands(DroneProgram &base_program,
                                       DroneProgram new_program);

  // Drone language visitors.
  bool Visit(GroundProgram *n, DroneProgram& drone_program);
  bool Visit(GroundCommand *n, DroneProgram& drone_program);
  bool Visit(WaypointCommand *n, DroneProgram& drone_program);
  bool Visit(UgvDropCommand *n, DroneProgram& drone_program);
  bool Visit(SurveyCommand *n, DroneProgram& drone_program);
  bool Visit(OffAxisCommand *n, DroneProgram& drone_program);
  bool Visit(WaitCommand *n, DroneProgram& drone_program);

  // Intermediate language visitors.
  bool Visit(GotoCommand *n, DroneProgram& drone_program);
};

} // namespace ground2drone_visitor
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
