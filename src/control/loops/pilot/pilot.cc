#include "pilot.h"

namespace spinny {
namespace control {
namespace loops {
namespace pilot {

PilotMissionHandler::PilotMissionHandler(MissionManager *mission_manager)
    : mission_manager_(mission_manager) {}

void PilotMissionHandler::operator()() {
  // Listen to ZMQ message queue and update MissionManager with any new missions
  // that come along.
  ::zmq::context_t context(1);
  ::zmq::socket_t mission_receiver_stream(context, ZMQ_REQ);
  mission_receiver_stream.connect("ipc:///tmp/mission_command_stream.ipc");

  while (true) {
    ::zmq::message_t mission_message;
    mission_receiver_stream.recv(&mission_message);
    ::std::string mission_message_string(
        static_cast<char *>(mission_message.data()), mission_message.size());

    ::spinny::control::mission_receiver::Mission mission_protobuf;
    mission_protobuf.ParseFromString(mission_message_string);

    mission_manager_->AddCommands(ParseMissionProtobuf(mission_protobuf));
  }
}

::std::vector<::std::shared_ptr<MissionCommand>>
PilotMissionHandler::ParseMissionProtobuf(
    ::spinny::control::mission_receiver::Mission mission_protobuf) {
  ::std::vector<::std::shared_ptr<MissionCommand>> new_commands;

  // Iterate through all commands in the protobuf and convert them to our C++
  // objects for commands.
  for (::spinny::control::mission_receiver::Command cmd_protobuf :
       mission_protobuf.commands()) {
    if (cmd_protobuf.type() == "GOTO") {
      new_commands.push_back(
          ::std::make_shared<MissionCommandGoto>(new MissionCommandGoto(
              cmd_protobuf.latitude(), cmd_protobuf.longitude(),
              cmd_protobuf.altitude())));

    } else if (cmd_protobuf.type() == "BOMB_DROP") {
      new_commands.push_back(::std::make_shared<MissionCommandBombDrop>(
          new MissionCommandBombDrop()));

    } else {
      // Return an empty command protobuf if we encounter any parsing errors
      // with the mission given.
      ::std::cerr << "ERROR: Invalid command type " << cmd_protobuf.type()
                  << ::std::endl;

      return ::std::vector<::std::shared_ptr<MissionCommand>>();
    }
  }

  return new_commands;
}

Pilot::Pilot() {}

PilotOutput Pilot::Calculate(Position3D drone_position) {
  ::std::shared_ptr<MissionCommand> cmd_ptr =
      mission_manager_.GetCurrentCommand();

  Vector3D flight_direction;
  bool bomb_drop = false;

  switch (cmd_ptr->type()) {
    case MissionCommand::GOTO: {
      constexpr double kSpeed = 15.0;

      ::std::shared_ptr<MissionCommandGoto> goto_cmd_ptr =
          ::std::static_pointer_cast<MissionCommandGoto>(cmd_ptr);

      Position3D goal = {goto_cmd_ptr->latitude(), goto_cmd_ptr->longitude(),
                         goto_cmd_ptr->altitude()};

      flight_direction = PointTowards(drone_position, goal);
      flight_direction *= kSpeed;

      break;
    }

    case MissionCommand::BOMB_DROP: {
      flight_direction = {0, 0, 0};
      bomb_drop = true;
      break;
    }

    case MissionCommand::DO_NOTHING: {
      flight_direction = {0, 0, 0};
      bomb_drop = false;
      break;
    }
  }

  return {flight_direction, bomb_drop};
}

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace spinny
