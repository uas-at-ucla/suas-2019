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
    ASSERT_FALSE(cmd->has_bombdropcommand());
    ASSERT_FALSE(cmd->has_gotocommand());

    mission_manager::GotoCommand *goto_cmd = cmd->mutable_gotocommand();
    mission_manager::Position3D *goal = goto_cmd->mutable_goal();
    goal->set_latitude(1.0 + i);
    goal->set_longitude(3.0 + i);
    goal->set_altitude(5.0 + i);

    ASSERT_FALSE(cmd->has_nothingcommand());
    ASSERT_FALSE(cmd->has_sleepcommand());
    ASSERT_FALSE(cmd->has_bombdropcommand());
    ASSERT_TRUE(cmd->has_gotocommand());
  }

  MissionManager mission_manager;
  mission_manager.SetCommands(mission);
  ASSERT_EQ(mission_manager.NumberOfCommands(), 100);

  for(int i = 0;i < 100;i++) {
    ::lib::mission_manager::Command cmd = mission_manager.GetCurrentCommand();
    mission_manager.PopCommand();
    ASSERT_EQ(cmd.gotocommand().goal().latitude(), 1.0 + i);
    ASSERT_EQ(cmd.gotocommand().goal().longitude(), 3.0 + i);
    ASSERT_EQ(cmd.gotocommand().goal().altitude(), 5.0 + i);
  }
}

TEST(MissionManagerTest, PreprocessorTest) {
  ::lib::mission_manager::Mission mission;

  for(int i = 0;i < 10;i++) {
    ::lib::mission_manager::Command *cmd = mission.add_commands();
    mission_manager::NothingCommand *nothing_cmd =
        cmd->mutable_nothingcommand();
    (void)nothing_cmd;
  }

  {
    ::lib::mission_manager::Command *cmd = mission.add_commands();
    mission_manager::WaypointCommand *waypoint_cmd =
        cmd->mutable_waypointcommand();
    (void)waypoint_cmd;
  }

  MissionManager mission_manager;
  mission_manager.SetCommands(mission);

  mission_manager.Preprocess();
  mission_manager.DumpMission();
}

}  // namespace testing
}  // namespace lib
