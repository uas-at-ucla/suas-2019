#include "context_visitor.h"

namespace src {
namespace controls {
namespace ground_server {
namespace timeline {
namespace context_visitors {
namespace {
constexpr double kLatitudeInf = 10000.;
}

ContextVisitor::ContextVisitor() {}

void ContextVisitor::Process(::std::string input) {
  GroundProgram input_program;
  try {
    if (input_program.ParseFromString(input)) {
      // if parsing is successful
      Visit(&input_program);
    } else {
      throw "cannot parse input string";
    }
  } catch (const char *msg) {
    throw msg;
  }
}

// Visitors ////////////////////////////////////////////////////////////////////
void ContextVisitor::Visit(GroundProgram *n) {
  // get the obstacles from n, store into visitor
  this->static_obstacles_ = n->static_obstacles();
  // get field boundary from n, store into visitor
  this->field_boundary_.CopyFrom(n->field_boundary());

  // Visit all commands.
  for (int i = 0; i < n->commands_size(); i++) {
    GroundCommand *current_command = n->mutable_commands(i);
    Visit(current_command);
  }
}

void ContextVisitor::Visit(GroundCommand *n) {
  // Select the specific command that this generic command type encloses, if
  // any.
  if (n->has_waypoint_command()) {
    Visit(n->mutable_waypoint_command());
  } else if (n->has_ugv_drop_command()) {
    Visit(n->mutable_ugv_drop_command());
  } else if (n->has_survey_command()) {
    Visit(n->mutable_survey_command());
  } else if (n->has_off_axis_command()) {
    Visit(n->mutable_off_axis_command());
  } else if (n->has_wait_command()) {
    Visit(n->mutable_wait_command());
  }
}

void ContextVisitor::Visit(WaypointCommand *n) {

  // Create a GotoCommand to fly to the waypoint while avoiding obstacles on the
  // field.
  {
    GotoCommand *goto_command = new GotoCommand();
    goto_command->mutable_goal()->CopyFrom(n->goal());
    if (!WithinBoundary(goto_command->mutable_goal())) {
      // if going out of bounds, throw exception
      throw "going out of bounds";
    }
    delete (goto_command);
  }
}

void ContextVisitor::Visit(UgvDropCommand *n) {
  // TODO(Comran): Implement
  (void)n;
}

void ContextVisitor::Visit(SurveyCommand *n) {

  // TODO(Weber): Implement
  (void)n;
}

void ContextVisitor::Visit(OffAxisCommand *n) {

  // TODO(Comran): Implement
  (void)n;
}

void ContextVisitor::Visit(WaitCommand *n) { (void)n; }

bool ContextVisitor::WithinBoundary(Position2D *p1, Position2D *p2) {
  ::google::protobuf::RepeatedPtrField<Position2D> field_boundary =
      this->field_boundary_;
  // check if the line defined by p1 and p2 is within the flight boundary
  if (field_boundary.size() < 3) {
    // must have at least 3 points to form a polygon
    return false;
  }
  if (WithinBoundary(p1) == false || WithinBoundary(p2) == false) {
    // segment is not in boundary if either endpoint is not in boundary
    return false;
  }
  for (int i = 0; i < field_boundary.size(); i++) {
    int next = (i + 1) % field_boundary.size();
    if (IfIntersect(p1, p2, field_boundary.Mutable(i),
                    field_boundary.Mutable(next))) {
      // if there's a intersection between i--next and p1--p2, line is outside
      // of boundary
      return false;
    }
  }
  return true;
}
bool ContextVisitor::WithinBoundary(Position3D *p) {
  // check if a 3D point is within the flight boundary, by converting it to a 2D
  // point, considering only latitude and longitude, assuming that the points in
  // flight boundary are given in CW order
  Position2D *p_2D = new Position2D();
  p_2D->set_latitude(p->latitude());
  p_2D->set_longitude(p->longitude());

  bool within = ContextVisitor::WithinBoundary(p_2D);
  delete (p_2D);

  return within;
}
bool ContextVisitor::WithinBoundary(Position2D *p) {
  // check if a 2D point is within the flight boundary
  // assuming that the points in flight boundary are given in CW order
  ::google::protobuf::RepeatedPtrField<Position2D> field_boundary =
      this->field_boundary_;

  if (field_boundary.size() < 3) {
    // must have at least 3 points to form a polygon
    return false;
  }

  // draw a point that has the same latitude as p but infinitely large longitude
  Position2D *extreme = new Position2D();
  extreme->set_latitude(kLatitudeInf);
  extreme->set_longitude(p->longitude());

  int count = 0; // count number of sides that cross with the horizontal line
                 // defined by p and extreme
  for (int i = 0; i < field_boundary.size(); i++) {
    int next = (i + 1) % field_boundary.size();
    if (IfIntersect(p, extreme, field_boundary.Mutable(i),
                    field_boundary.Mutable(next))) {
      // if there's a intersection between i--next and p--extreme
      count++;
    }
  }
  delete (extreme);
  // returns true if number of intersections is odd
  if (count % 2 == 1) {
    return true;
  }
  return false;
}
bool ContextVisitor::IfIntersect(Position2D *p1, Position2D *q1, Position2D *p2,
                                 Position2D *q2) {
  // check if the line given by p1, q1 and line given by p2, q2 intersect
  // overlapping lines are seen as non-intersecting
  // modified code of:
  // https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

  // Find the four orientations needed for general and
  // special cases
  int o1 = Orientation(p1, q1, p2);
  int o2 = Orientation(p1, q1, q2);
  int o3 = Orientation(p2, q2, p1);
  int o4 = Orientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4)
    return true;

  return false; // Doesn't fall in any of the above cases
}
int ContextVisitor::Orientation(Position2D *a, Position2D *b, Position2D *c) {
  // determine the orientation given by the triplet a, b, c
  // 0: a, b, c are collinear
  // 1: CW
  // 2: CCW
  // modified code of:
  // https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
  double value =
      (b->longitude() - a->longitude()) * (c->latitude() - b->latitude()) -
      (c->longitude() - b->longitude()) * (b->latitude() - a->latitude());
  if (value == 0)
    return 0;

  return (value > 0) ? 1 : 2;
}
} // namespace context_visitors
} // namespace timeline
} // namespace ground_server
} // namespace controls
} // namespace src
