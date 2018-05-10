#include "pilot.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {
namespace {

// Constant for how tight we want the ramp to be for re-centering the drone
// on the straight-line path between start and end.
constexpr double kGotoRawReCenterRamp = 15;

// Required tolerance distance, in meters, for halting goto_raw waypoints to
// trigger a continuation to the next command.
constexpr double kGotoRawWaypointTolerance = 3;

// Gains for slowing down the drone when it needs to come to a halt at a
// position.
constexpr double kGotoRawWaypointPIDProportionalityGain = 0.4;
}  // namespace

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
      last_cmd_ = cmd_;
    }
  } else {
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
    constexpr double kSpeed = 25.0;

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

    /*
    //       drone
    //         \\
    //          \\
    //           \\______________
    //                           \_______
    // origin                            \--------------------------------- goal
    */

    Position3D start = {last_cmd_.gotorawcommand().goal().latitude(),
                        last_cmd_.gotorawcommand().goal().longitude(),
                        last_cmd_.gotorawcommand().goal().altitude()};
    Position3D end = {cmd_.gotorawcommand().goal().latitude(),
                      cmd_.gotorawcommand().goal().longitude(),
                      cmd_.gotorawcommand().goal().altitude()};

    /*
    //                  drone
    //                    |\
    //                    | \
    //                    |  \
    //                    |   \
    //                    |    \
    //                 e  |     \
    //                 r  |      \
    //                 r  |       \
    //                 o  |        \
    //                 r  |         \
    //                    |          \
    //                    |           \
    //                    |            \
    //                    |  projected  \
    // start *************---------------- end
    //
    */

    // Vectors are in NED coordinates, which means altitude components are
    // flipped...
    ::Eigen::Vector3d drone_to_end_vector(
        GetDistance2D({0, 0, 0}, {1, 0, 0}) *
            (end.latitude - drone_position.latitude),
        GetDistance2D({0, 0, 0}, {0, 1, 0}) *
            (end.longitude - drone_position.longitude),
        drone_position.altitude - end.altitude);

    ::Eigen::Vector3d start_to_end_vector(
        GetDistance2D({0, 0, 0}, {1, 0, 0}) * (end.latitude - start.latitude),
        GetDistance2D({0, 0, 0}, {0, 1, 0}) * (end.longitude - start.longitude),
        start.altitude - end.altitude);

    ::Eigen::Vector3d drone_projection_on_path_vector =
        start_to_end_vector * start_to_end_vector.dot(drone_to_end_vector) /
        start_to_end_vector.dot(start_to_end_vector);

    ::Eigen::Vector3d error_vector =
        (drone_to_end_vector - drone_projection_on_path_vector) /
        kGotoRawReCenterRamp;

    // A value, between 0 and 1, which determines whether the output vector
    // should point straight at the goal relative to the start-end path (mix =
    // 0) or if the output vector should point perpendicular to the path to
    // re-align the drone with the route.
    double mix = ::std::min(1.0, ::std::pow(error_vector.norm(), 2));

    ::Eigen::Vector3d flight_direction_vector =
        (drone_projection_on_path_vector /
         ::std::max(1.0, drone_projection_on_path_vector.norm())) *
            (1 - mix) +
        (error_vector / ::std::max(1.0, error_vector.norm())) * mix;

    // Convert from Eigen vectors to our code's internal Vector3D structure.
    flight_direction = {flight_direction_vector.x(),
                        flight_direction_vector.y(),
                        flight_direction_vector.z()};

    if (cmd.gotorawcommand().come_to_stop()) {
      flight_direction *=
          ::std::min(kSpeed,
                     drone_projection_on_path_vector.norm() *
                         kGotoRawWaypointPIDProportionalityGain);
    } else {
      flight_direction *= kSpeed;
    }

    // Stores whether the drone is now on the other side of the endpoint after
    // having started on the other side.
    bool passed_endpoint = start_to_end_vector.dot(drone_to_end_vector) < 0;

    // Stores whether the drone will meet the goal very soon at its current
    // speed.
    bool will_meet_goal_in_near_future = drone_to_end_vector.norm() < kSpeed;

    // Decide whether or not to move to the next command.
    if ((cmd.gotorawcommand().come_to_stop() &&
         drone_projection_on_path_vector.norm() < kGotoRawWaypointTolerance) ||
        (!cmd.gotorawcommand().come_to_stop() &&
         (passed_endpoint || will_meet_goal_in_near_future))) {
      mission_message_queue_receiver_.get_mission_manager()->PopCommand();
    }
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
