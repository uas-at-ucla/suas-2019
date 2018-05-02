#include "pilot.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {

Pilot::Pilot()
    : cmd_set_(false),
      drone_position_set_(false),
      drone_position_semaphore_(1),
      thread_(&Pilot::PreprocessorThread, this) {}

Pilot::~Pilot() {
  Quit();
  thread_.join();
}

void Pilot::PreprocessorThread() {
  while (run_) {
    if (drone_position_set_) {
      drone_position_semaphore_.Wait();
      Position3D drone_position = drone_position_;
      drone_position_semaphore_.Notify();

      mission_message_queue_receiver_.RunPreprocessor(drone_position);
    }

    usleep(1e4);
  }
}

PilotOutput Pilot::Calculate(Position3D drone_position) {
  drone_position_semaphore_.Wait();

  drone_position_ = drone_position;
  drone_position_set_ = true;
  drone_position_semaphore_.Notify();

  ::lib::mission_manager::Command cmd =
      mission_message_queue_receiver_.get_mission_manager()
          ->GetCurrentCommand();

  // Keep track of the current and previous commands for the current mission.
  if (cmd_set_) {
    if (!google::protobuf::util::MessageDifferencer::Equals(cmd, cmd_)) {
      ::std::cout << "DIFFERENCE------------------------------------------\n";
      last_cmd_ = cmd_;
    }
  } else {
    ::std::cout << "NOTHING------------------------\n";
    last_cmd_.mutable_nothingcommand();
    cmd_set_ = true;
  }

  cmd_ = cmd;

  Vector3D flight_direction = {0, 0, 0};
  bool bomb_drop = false;

  if (cmd.has_nothingcommand()) {
    // Do nothing.
    mission_message_queue_receiver_.get_mission_manager()->PopCommand();
  } else if (cmd.has_sleepcommand()) {
    // Sleep.
  } else if (cmd.has_gotorawcommand()) {
    // Go directly to a location on a field.
    constexpr double kSpeed = 12.0;

    if (!last_cmd_.has_gotorawcommand()) {
      // Create a mock goto_raw location at the drone's current position to make
      // the following drone routing algorithm work.

      ::lib::mission_manager::Position3D *goal =
          last_cmd_.mutable_gotorawcommand()->mutable_goal();
      goal->set_latitude(drone_position.latitude);
      goal->set_longitude(drone_position.longitude);
      goal->set_altitude(drone_position.altitude);
    }

    // Funneling algorithm for keeping the drone on a defined path on its way to
    // the goal.

    //       drone
    //         \\
    //          \\
    //           \\______________
    //                           \_______
    // origin                            \--------------------------------- goal

    Position3D goal = {cmd_.gotorawcommand().goal().latitude(),
                       cmd_.gotorawcommand().goal().longitude(),
                       cmd_.gotorawcommand().goal().altitude()};
    Position3D last_goal = {last_cmd_.gotorawcommand().goal().latitude(),
                            last_cmd_.gotorawcommand().goal().longitude(),
                            last_cmd_.gotorawcommand().goal().altitude()};

    //                  drone
    //                    |\
    //                    | \
    //                    |  \
    //                    |   \
    //                    |    \
    //                    |   distance
    //                    |      \
    //                    |       \
    //                 deviation   \
    //                    |         \
    //                    |          \
    //                    |           \
    //                    |            \
    //                    |             \
    //                    |              \
    // origin  ...path... |-- projected --\ goal
    //                        distance
    ::Eigen::Vector3d distance_vector(
        GetDistance2D({0, 0, 0}, {0, 1, 0}) *
            (goal.longitude - drone_position.longitude),
        GetDistance2D({0, 0, 0}, {1, 0, 0}) *
            (goal.latitude - drone_position.latitude),
        drone_position.altitude - goal.altitude);

    ::Eigen::Vector3d path_vector(GetDistance2D({0, 0, 0}, {0, 1, 0}) *
                                      (goal.longitude - last_goal.longitude),
                                  GetDistance2D({0, 0, 0}, {1, 0, 0}) *
                                      (goal.latitude - last_goal.latitude),
                                  goal.altitude - last_goal.altitude);
    ::std::cout << "PATH VECTOR: " << path_vector << ::std::endl;

    double distance = distance_vector.norm();

    ::Eigen::Vector3d path_projection = path_vector *
                                        path_vector.dot(distance_vector) /
                                        path_vector.dot(path_vector);

    ::Eigen::Vector3d error = distance_vector - path_projection;
    ::std::cout << error << ::std::endl;

    ::Eigen::Vector3d adjustment = error / 15;
    double mix = ::std::min(1.0, ::std::pow(adjustment.norm(), 1.5));

    ::Eigen::Vector3d flight_direction_vector =
        (path_vector / path_vector.norm()) * (1 - mix) +
        (adjustment / adjustment.norm()) * (mix);

    double projected_distance = path_projection.norm();

    ::std::cout << "distance: " << distance << ::std::endl;
    ::std::cout << "projected: " << projected_distance << ::std::endl;
    ::std::cout << "mix: " << mix << ::std::endl;
    ::std::cout << "adjustment: " << adjustment.norm() << ::std::endl;

    // 0 is aim at goal, 1 is move drone towards path.
    ::std::cout << flight_direction_vector << ::std::endl;

    flight_direction = {flight_direction_vector.y(),
                        flight_direction_vector.x(),
                        flight_direction_vector.z()};
    flight_direction *= kSpeed;

    if (GetDistance2D(drone_position, goal) < kSpeed) {
      mission_message_queue_receiver_.get_mission_manager()->PopCommand();
    }
  } else if (cmd.has_bombdropcommand()) {
    flight_direction = {0, 0, 0};
    bomb_drop = true;
  } else {
    ::std::cout << "ERROR: Unknown command.\n";
  }

  return {flight_direction, bomb_drop};
}

void Pilot::SetMission(::lib::mission_manager::Mission mission) {
  mission_message_queue_receiver_.SetMission(mission);
}

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src
