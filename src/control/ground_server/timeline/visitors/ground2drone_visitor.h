#pragma once

#include "src/control/ground_server/timeline/timeline_grammar.pb.h"

namespace src {
namespace control {
namespace ground_server {
namespace timeline {
namespace visitors {

class Ground2DroneVisitor {
 public:
  void Visit(::src::control::ground_server::timeline::GroundProgram n);

 private:
  void Visit(::src::control::ground_server::timeline::GroundCommand n);
  void Visit(::src::control::ground_server::timeline::WaypointCommand n);
  void Visit(::src::control::ground_server::timeline::BombDropCommand n);
  void Visit(::src::control::ground_server::timeline::SurveyCommand n);
  void Visit(::src::control::ground_server::timeline::OffAxisCommand n);
};

} // namespace visitors
} // namespace timeline
} // namespace ground_server
} // namespace control
} // namespace src
