#include "pilot.h"

namespace src {
namespace control {
namespace loops {
namespace pilot {

PilotMissionHandler::PilotMissionHandler(::lib::MissionManager *mission_manager)
    : mission_manager_(mission_manager) {}

void PilotMissionHandler::operator()() {
  // Listen to ZMQ message queue and update MissionManager with any new missions
  // that come along.
  ::zmq::context_t context(1);
  ::zmq::socket_t ground_communicator_stream(context, ZMQ_REQ);
  ground_communicator_stream.connect("ipc:///tmp/mission_command_stream.ipc");

  while (true) {
    ::zmq::message_t mission_message;
    ground_communicator_stream.recv(&mission_message);
    ::std::string mission_message_string(
        static_cast<char *>(mission_message.data()), mission_message.size());

    ::src::controls::ground_communicator::Mission mission_protobuf;
    mission_protobuf.ParseFromString(mission_message_string);

    mission_manager_->AddCommands(ParseMissionProtobuf(mission_protobuf));
  }
}

::std::vector<::std::shared_ptr<::lib::MissionCommand>>
PilotMissionHandler::ParseMissionProtobuf(
    ::src::controls::ground_communicator::Mission mission_protobuf) {
  ::std::vector<::std::shared_ptr<::lib::MissionCommand>> new_commands;

  // Iterate through all commands in the protobuf and convert them to our C++
  // objects for commands.
  for (::src::controls::ground_communicator::Command cmd_protobuf :
       mission_protobuf.commands()) {
    if (cmd_protobuf.type() == "goto") {
      new_commands.push_back(::std::make_shared<::lib::MissionCommandGoto>(
          new ::lib::MissionCommandGoto(cmd_protobuf.latitude(),
                                        cmd_protobuf.longitude(),
                                        cmd_protobuf.altitude())));

    } else if (cmd_protobuf.type() == "bomb") {
      new_commands.push_back(::std::make_shared<::lib::MissionCommandBombDrop>(
          new ::lib::MissionCommandBombDrop()));

    } else {
      // Return an empty command protobuf if we encounter any parsing errors
      // with the mission given.
      ::std::cerr << "ERROR: Invalid command type " << cmd_protobuf.type()
                  << ::std::endl;

      return ::std::vector<::std::shared_ptr<::lib::MissionCommand>>();
    }
  }

  return new_commands;
}

Pilot::Pilot() : pilot_mission_handler_(&mission_manager_) {}

PilotOutput Pilot::Calculate(Position3D drone_position) {
  ::std::shared_ptr<::lib::MissionCommand> cmd_ptr =
      mission_manager_.GetCurrentCommand();

  Vector3D flight_direction;
  bool bomb_drop = false;

  switch (cmd_ptr->type()) {
    case ::lib::MissionCommand::GOTO: {
      constexpr double kSpeed = 15.0;

      ::std::shared_ptr<::lib::MissionCommandGoto> goto_cmd_ptr =
          ::std::static_pointer_cast<::lib::MissionCommandGoto>(cmd_ptr);

      Position3D goal = {goto_cmd_ptr->latitude(), goto_cmd_ptr->longitude(),
                         goto_cmd_ptr->altitude()};

      flight_direction = PointTowards(drone_position, goal);
      flight_direction *= kSpeed;

      break;
    }

    case ::lib::MissionCommand::BOMB_DROP: {
      flight_direction = {0, 0, 0};
      bomb_drop = true;
      break;
    }

    case ::lib::MissionCommand::DO_NOTHING: {
      flight_direction = {0, 0, 0};
      bomb_drop = false;
      break;
    }
  }

  return {flight_direction, bomb_drop};
}

void Pilot::HandleMission() {
  ::std::cerr << "Able to handle mission from mission_handler" << ::std::endl;
  pilot_mission_handler_();
}

}  // namespace pilot
}  // namespace loops
}  // namespace control
}  // namespace src
