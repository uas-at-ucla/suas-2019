#pragma once

#include "src/controls/ground_server/timeline/timeline_grammar.pb.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace ground2drone_visitor {

class Ground2DroneVisitor {
 public:
  Ground2DroneVisitor();
  DroneProgram Process(GroundProgram *input_program);

 private:
  void ConcatenateDroneProgramCommands(DroneProgram &base_program,
                                       DroneProgram new_program);

  // Drone language visitors.
  DroneProgram Visit(GroundProgram *n);
  DroneProgram Visit(GroundCommand *n);
  DroneProgram Visit(WaypointCommand *n);
  DroneProgram Visit(UgvDropCommand *n);
  DroneProgram Visit(SurveyCommand *n);
  DroneProgram Visit(OffAxisCommand *n);
  DroneProgram Visit(WaitCommand *n);

  // Intermediate language visitors.
  DroneProgram Visit(GotoCommand *n);
};

} // namespace ground2drone_visitor
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
