#include "gtest/gtest.h"

#include "lib/mission_manager/mission_manager.h"

namespace lib {
namespace testing {

TEST(MissionManagerTest, MissionManagerTest) {
  ::lib::mission_manager::Mission mission;

  for (int i = 0; i < 100; i++) {
    ::lib::mission_manager::Command *cmd = mission.add_commands();

    mission_manager::NothingCommand *nothing_cmd =
        cmd->mutable_nothingcommand();
    (void)nothing_cmd;

    ASSERT_TRUE(cmd->has_nothingcommand());
    ASSERT_FALSE(cmd->has_sleepcommand());
    ASSERT_FALSE(cmd->has_bombcommand());
    ASSERT_FALSE(cmd->has_gotocommand());

    mission_manager::GotoCommand *goto_cmd = cmd->mutable_gotocommand();
    goto_cmd->set_latitude(1.0 + i);
    goto_cmd->set_longitude(3.0 + i);
    goto_cmd->set_altitude(5.0 + i);

    ASSERT_FALSE(cmd->has_nothingcommand());
    ASSERT_FALSE(cmd->has_sleepcommand());
    ASSERT_FALSE(cmd->has_bombcommand());
    ASSERT_TRUE(cmd->has_gotocommand());
  }

  MissionManager mission_manager;
  mission_manager.SetCommands(mission);
  ASSERT_EQ(mission_manager.NumberOfCommands(), 100);

  for(int i = 0;i < 100;i++) {
    ::lib::mission_manager::Command cmd = mission_manager.GetCurrentCommand();
    mission_manager.PopCommand();
    ASSERT_EQ(cmd.gotocommand().latitude(), 1.0 + i);
    ASSERT_EQ(cmd.gotocommand().longitude(), 3.0 + i);
    ASSERT_EQ(cmd.gotocommand().altitude(), 5.0 + i);
  }
}

}  // namespace testing
}  // namespace lib
