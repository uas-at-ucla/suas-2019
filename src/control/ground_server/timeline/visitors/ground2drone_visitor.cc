#include "ground2drone_visitor.h"

namespace src {
namespace control {
namespace ground_server {
namespace timeline {
namespace visitors {

void Ground2DroneVisitor::Visit(
    ::src::control::ground_server::timeline::GroundProgram n) {
  (void) n;
}

void Ground2DroneVisitor::Visit(
    ::src::control::ground_server::timeline::GroundCommand n) {
  (void) n;
}

void Ground2DroneVisitor::Visit(
    ::src::control::ground_server::timeline::WaypointCommand n) {
  (void) n;
}

void Ground2DroneVisitor::Visit(
    ::src::control::ground_server::timeline::BombDropCommand n) {
  (void) n;
}

void Ground2DroneVisitor::Visit(
    ::src::control::ground_server::timeline::SurveyCommand n) {
  (void) n;
}

void Ground2DroneVisitor::Visit(
    ::src::control::ground_server::timeline::OffAxisCommand n) {
  (void) n;
}

} // namespace visitors
} // namespace timeline
} // namespace ground_server
} // namespace control
} // namespace src
