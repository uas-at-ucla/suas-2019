#pragma once

#include "src/controls/ground_server/timeline/timeline_grammar.pb.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace context_visitors {

class ContextVisitor {
 public:
  ::google::protobuf::RepeatedPtrField<StaticObstacle> static_obstacles;
  ::google::protobuf::RepeatedPtrField<Position2D> field_boundary;
  ContextVisitor();
  void Process(::std::string input);
  // helper functions
  bool WithinBoundary(Position2D *p1, Position2D *p2);
  bool WithinBoundary(Position2D *p);
  bool WithinBoundary(Position3D *p);

 private:
  // Drone language visitors.
  void Visit(GroundProgram *n);
  void Visit(GroundCommand *n);
  void Visit(WaypointCommand *n);
  void Visit(UgvDropCommand *n);
  void Visit(SurveyCommand *n);
  void Visit(OffAxisCommand *n);
  void Visit(WaitCommand *n);

  // helper variables
  const double LATITUDE_INF = 10000.;

  // HELPER FUNCTIONS
  bool IfIntersect(Position2D *p1, Position2D *p2, Position2D *q1,
                   Position2D *q2);
  int Orientation(Position2D *a, Position2D *b, Position2D *c);

  // Intermediate language visitors.
  void Visit(GotoCommand *n);
};

} // namespace context_visitors
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
