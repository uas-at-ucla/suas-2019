#include "gtest/gtest.h"

#include "lib/mission_manager/mission_manager.h"

namespace spinny {
namespace testing {

TEST(MissionManagerTest, CommandsTest) {
  // Goto cmd
  {
    double lat = 1.1;
    double lng = 2.1;
    double alt = 3.1;

    MissionCommandGoto goto_cmd(lat, lng, alt);

    ASSERT_EQ(goto_cmd.type(), MissionCommand::GOTO);
    ASSERT_EQ(goto_cmd.latitude(), lat);
    ASSERT_EQ(goto_cmd.longitude(), lng);
    ASSERT_EQ(goto_cmd.altitude(), alt);
  }

  // Bomb cmd
  {
    MissionCommandBombDrop bomb_cmd;
    ASSERT_EQ(bomb_cmd.type(), MissionCommand::BOMB_DROP);
  }
}

TEST(MissionManagerTest, MissionManagerTest) {
  double lat = 1.1;
  double lng = 2.1;
  double alt = 3.1;

  MissionManager mission_manager;

  ::std::vector<::std::shared_ptr<MissionCommand>> commands_to_add;

  for (int i = 0; i < 1000; i++) {
    commands_to_add.push_back(::std::make_shared<MissionCommandGoto>(
        new MissionCommandGoto(lat + i, lng + i * 2, alt + i * 3)));
  }

  for (int i = 0; i < 1000; i++) {
    commands_to_add.push_back(::std::make_shared<MissionCommandBombDrop>(
        new MissionCommandBombDrop()));
  }

  ASSERT_EQ(mission_manager.NumberOfCommands(), 0);
  mission_manager.AddCommands(commands_to_add);
  ASSERT_EQ(mission_manager.NumberOfCommands(), 2000);

  for (int i = 0; i < 2000; i++) {
    ::std::shared_ptr<MissionCommand> cmd_ptr =
        mission_manager.GetCurrentCommand();

    if(i < 1000) {
      ASSERT_EQ(cmd_ptr->type(), MissionCommand::GOTO);
    } else {
      ASSERT_EQ(cmd_ptr->type(), MissionCommand::BOMB_DROP);
    }

    if (cmd_ptr.get()->type() == MissionCommand::GOTO) {
      ::std::shared_ptr<MissionCommandGoto> goto_cmd_ptr =
          ::std::static_pointer_cast<MissionCommandGoto>(cmd_ptr);

      ASSERT_EQ(goto_cmd_ptr->latitude(), lat + i);
      ASSERT_EQ(goto_cmd_ptr->longitude(), lng + i * 2);
      ASSERT_EQ(goto_cmd_ptr->altitude(), alt + i * 3);
    } else if(cmd_ptr->type() == MissionCommand::BOMB_DROP) {
      ::std::shared_ptr<MissionCommandBombDrop> bomb_drop_cmd_ptr =
          ::std::static_pointer_cast<MissionCommandBombDrop>(cmd_ptr);
    }

    mission_manager.PopCommand();
  }

  mission_manager.ClearCommands();

  ASSERT_EQ(mission_manager.NumberOfCommands(), 0);
}

}  // namespace testing
}  // namespace spinny
