#pragma once

#include "src/controls/ground_server/timeline/timeline_grammar.pb.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace context_visitors {

class ContextVisitor {
 public:
  ContextVisitor();
  void Process(::std::string input);

 private:
  // Drone language visitors.
  void Visit(GroundProgram *n);
  void Visit(GroundCommand *n);
  void Visit(WaypointCommand *n);
  void Visit(UgvDropCommand *n);
  void Visit(SurveyCommand *n);
  void Visit(OffAxisCommand *n);
  void Visit(WaitCommand *n);

  // variables that describe the flight
  ::google::protobuf::RepeatedPtrField<StaticObstacle> static_obstacles_;
  ::google::protobuf::RepeatedPtrField<Position2D> field_boundary_;

  // helper functions
  bool WithinBoundary(Position2D *p1, Position2D *p2);
  bool WithinBoundary(Position2D *p);
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
