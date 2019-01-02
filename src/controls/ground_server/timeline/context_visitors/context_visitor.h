#pragma once

#include "lib/mission_manager/mission_commands.pb.h"
#include "lib/rrt_avoidance/rrt_avoidance.h"
#include "src/controls/ground_server/timeline/timeline_grammar.pb.h"

// using protobuf messages defined in this namespace
using namespace lib::mission_manager;

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace context_visitors {

class ContextVisitor {
 public:
  ContextVisitor();
  void Process(::std::string input, Position3D drone_position);
  ::std::vector<Position3D> getAvoidancePath();
 private:
  // Drone language visitors.
  void Visit(GroundProgram *n, Position3D drone_position);
  void Visit(GroundCommand *n, Position3D drone_position);
  void Visit(WaypointCommand *n, Position3D drone_position);
  void Visit(UgvDropCommand *n);
  void Visit(SurveyCommand *n);
  void Visit(OffAxisCommand *n);
  void Visit(WaitCommand *n);
  void Visit(GotoCommand *n, Position3D drone_position);

  // variables that describe the flight
  ::google::protobuf::RepeatedPtrField<StaticObstacle> static_obstacles_;
  ::google::protobuf::RepeatedPtrField<Position2D> field_boundary_;

  //calculated avoidance path
  ::std::vector<Position3D> avoidance_path_;
  
 

  // helper functions
  bool WithinBoundary(Position2D *p1, Position2D *p2);
  bool WithinBoundary(Position2D *p);
  bool WithinBoundary(Position3D *p1, Position3D *p2);
  bool WithinBoundary(Position3D *p);
  bool IfIntersect(Position2D *p1, Position2D *p2, Position2D *q1,
                   Position2D *q2);
  int Orientation(Position2D *a, Position2D *b, Position2D *c);
};

} // namespace context_visitors
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
