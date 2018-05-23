#include "pilot.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {
namespace {

constexpr double kSpeed = 25.0;

// Constant for how tight we want the ramp to be for re-centering the drone
// on the straight-line path between start and end.
constexpr double kGotoRawReCenterRamp = 15;

// Required tolerance distance, in meters, for halting goto_raw waypoints to
// trigger a continuation to the next command.
constexpr double kGotoRawWaypointTolerance = 1.3;

} // namespace

Pilot::Pilot()
    : thrust_pid_(1 / 100.0, kSpeed, 0, 0.3, 0.4, 0, 4), cmd_set_(false),
      drone_position_set_(false), drone_position_semaphore_(1),
      thread_(&Pilot::PreprocessorThread, this), sleep_time_(0),
      come_to_stop_(true), come_to_stop_count_(0), setpoint_reset_(true),
      met_goal_(false) {}

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

Vector3D Pilot::VelocityNavigator() {
  if (setpoint_reset_) {
    come_to_stop_count_ = 0;
    met_goal_ = false;
    setpoint_reset_ = false;
  }

  // Go directly to a location on a field.

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
          (end_.latitude - drone_position_.latitude),
      GetDistance2D({0, 0, 0}, {0, 1, 0}) *
          (end_.longitude - drone_position_.longitude),
      drone_position_.altitude - end_.altitude);

  ::Eigen::Vector3d start_to_end_vector(
      GetDistance2D({0, 0, 0}, {1, 0, 0}) * (end_.latitude - start_.latitude),
      GetDistance2D({0, 0, 0}, {0, 1, 0}) * (end_.longitude - start_.longitude),
      start_.altitude - end_.altitude);

  ::Eigen::Vector3d drone_projection_on_path_vector =
      start_to_end_vector.norm() > 0
          ? start_to_end_vector * start_to_end_vector.dot(drone_to_end_vector) /
                start_to_end_vector.dot(start_to_end_vector)
          : ::Eigen::Vector3d(0, 0, 0);

  ::Eigen::Vector3d error_vector =
      (drone_to_end_vector - drone_projection_on_path_vector) /
      kGotoRawReCenterRamp;

  // A value, between 0 and 1, which determines whether the output vector
  // should point straight at the goal relative to the start-end path (mix =
  // 0) or if the output vector should point perpendicular to the path to
  // re-align the drone with the route.

  ::Eigen::Vector3d flight_direction_vector;

  // A vector for directing the drone towards its goal point while also
  // keeping it roughly close to a direct path between its start and end.
  double half_pipe_mix = ::std::min(1.0, ::std::pow(error_vector.norm(), 2));
  ::Eigen::Vector3d flight_direction_half_pipe_vector =
      ((drone_projection_on_path_vector /
        ::std::max(1.0, drone_projection_on_path_vector.norm())) *
           (1 - half_pipe_mix) +
       (error_vector / ::std::max(1.0, error_vector.norm())) * half_pipe_mix) *
      kSpeed;

  // A vector for directing the drone towards its goal point from all
  // directions.
  double thrust_factor =
      ::std::min(thrust_pid_.calculate(0, -drone_to_end_vector.norm()), kSpeed);
  ::Eigen::Vector3d flight_direction_black_hole_vector =
      drone_to_end_vector / ::std::max(1.0, drone_to_end_vector.norm()) *
      thrust_factor;

  // Choose the method for moving towards the goal based off whether we want
  // to halt at the end or continue going to another waypoint.

  if (come_to_stop_) {
    // Fade between half-pipe and black hole methods for routing the drone in
    // the final stretch between the drone reaching the goal when it intends
    // to stop.
    double mix = ::std::max(
        0.0, ::std::min(1.0, (drone_to_end_vector.norm() - 15) / 30));

    flight_direction_vector = flight_direction_half_pipe_vector * mix +
                              flight_direction_black_hole_vector * (1 - mix);
  } else {
    flight_direction_vector = flight_direction_half_pipe_vector;
  }

  // Convert from Eigen vectors to our code's internal Vector3D structure.
  // TODO(comran): Use Eigen Vector3D for everything.
  Vector3D flight_direction = {flight_direction_vector.x(),
                               flight_direction_vector.y(),
                               flight_direction_vector.z()};

  // Stores whether the drone is now on the other side of the endpoint after
  // having started on the other side.
  bool passed_endpoint = start_to_end_vector.dot(drone_to_end_vector) < 0;

  // Stores whether the drone will meet the goal very soon at its current
  // speed.
  bool will_meet_goal_in_near_future = drone_to_end_vector.norm() < kSpeed;

  // Decide whether or not to move to the next command.
  if (!met_goal_) {
    if (come_to_stop_) {
      if (drone_to_end_vector.norm() < kGotoRawWaypointTolerance) {
        come_to_stop_count_++;
      }

      if (come_to_stop_count_ > 100 * 2) {
        // If the drone has been at its goal for more than two seconds, move on.
        met_goal_ = true;
      }
    } else if (!come_to_stop_ &&
               (passed_endpoint || will_meet_goal_in_near_future)) {
      met_goal_ = true;
    }
  }

  return flight_direction;
}

bool Pilot::MetGoal() { return met_goal_ && !setpoint_reset_; }

PilotOutput Pilot::Calculate(Position3D drone_position) {
  if (!drone_position_set_) {
    start_ = drone_position;
    end_ = drone_position;
  }

  drone_position_semaphore_.Wait();
  drone_position_ = drone_position;
  drone_position_set_ = true;
  drone_position_semaphore_.Notify();

  ::lib::mission_manager::Command cmd =
      mission_message_queue_receiver_.get_mission_manager()
          ->GetCurrentCommand();

  // Keep track of the current and previous commands for the current mission.
  // TODO(comran): This is not a good implementation (what if the commands are
  //                exactly the same?)
  come_to_stop_ = true;

  if (cmd_set_) {
    if (!google::protobuf::util::MessageDifferencer::Equals(cmd, cmd_)) {
      if (cmd.has_gotorawcommand()) {
        setpoint_reset_ = true;
      }

      last_cmd_ = cmd_;
    }
  } else {
    last_cmd_.mutable_nothingcommand();
    cmd_set_ = true;
  }

  cmd_ = cmd;

  bool bomb_drop = false;

  if (cmd.has_nothingcommand()) {
    // Do nothing.
    mission_message_queue_receiver_.get_mission_manager()->PopCommand();
  } else if (cmd.has_sleepcommand()) {
    // Sleep.
    if (mission_message_queue_receiver_.get_mission_manager()
            ->CheckNewCommand()) {
      sleep_time_ = 0;
    } else {
      sleep_time_ +=
          1 / 100.0; // TODO(comran): Loop speed should not be hard coded.
    }

    if (sleep_time_ >= cmd.sleepcommand().time()) {
      mission_message_queue_receiver_.get_mission_manager()->PopCommand();
    }

    ::std::cout << sleep_time_ << " / " << cmd.sleepcommand().time()
                << ::std::endl;
  } else if (cmd.has_gotorawcommand()) {
    start_ = end_;

    end_ = {cmd_.gotorawcommand().goal().latitude(),
            cmd_.gotorawcommand().goal().longitude(),
            cmd_.gotorawcommand().goal().altitude()};

    come_to_stop_ = cmd.gotorawcommand().come_to_stop();

    if (MetGoal()) {
      mission_message_queue_receiver_.get_mission_manager()->PopCommand();
    }
  } else {
    ::std::cout << "ERROR: Unknown command.\n";
  }

  Vector3D flight_direction = VelocityNavigator();

  return {flight_direction, bomb_drop};
}

void Pilot::SetMission(::lib::mission_manager::Mission mission) {
  mission_message_queue_receiver_.SetMission(mission);
}

} // namespace pilot
} // namespace loops
} // namespace control
} // namespace src
