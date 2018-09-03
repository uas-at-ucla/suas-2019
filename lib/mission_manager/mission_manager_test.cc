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

  for (int i = 0; i < 100; i++) {
    ::lib::mission_manager::Command cmd = mission_manager.GetCurrentCommand();
    mission_manager.PopCommand();
    ASSERT_EQ(cmd.gotocommand().goal().latitude(), 1.0 + i);
    ASSERT_EQ(cmd.gotocommand().goal().longitude(), 3.0 + i);
    ASSERT_EQ(cmd.gotocommand().goal().altitude(), 5.0 + i);
  }
}

// TODO(comran): Re-enable when we add back in the goto raw avoidance stuff.

// TEST(MissionManagerTest, PreprocessorTest) {
//  ::lib::mission_manager::Mission mission;

//  for (int i = 0; i < 10; i++) {
//    ::lib::mission_manager::Command *cmd = mission.add_commands();
//    mission_manager::NothingCommand *nothing_cmd =
//        cmd->mutable_nothingcommand();
//    (void)nothing_cmd;
//  }

//  {
//    mission_manager::WaypointCommand *waypoint_cmd =
//        mission.add_commands()->mutable_waypointcommand();

//    ::lib::mission_manager::Position3D *goal =
//        new ::lib::mission_manager::Position3D();

//    goal->set_latitude(0.01);
//    goal->set_longitude(0.01);
//    goal->set_altitude(10);

//    waypoint_cmd->set_allocated_goal(goal);
//  }

//  {
//    mission_manager::BombDropCommand *bomb_cmd =
//        mission.add_commands()->mutable_bombdropcommand();

//    ::lib::mission_manager::Position2D *drop_zone =
//        new ::lib::mission_manager::Position2D();

//    drop_zone->set_latitude(0.01);
//    drop_zone->set_longitude(0.01);

//    bomb_cmd->set_allocated_drop_zone(drop_zone);
//  }

//  ::lib::mission_manager::Obstacles obstacles;
//  ::lib::mission_manager::StaticObstacle *obstacle =
//      obstacles.add_static_obstacles();
//  ::lib::mission_manager::Position2D *location =
//      new ::lib::mission_manager::Position2D();

//  location->set_latitude(0.005);
//  location->set_longitude(0.005);
//  obstacle->set_allocated_location(location);

//  obstacle->set_cylinder_radius(100);

//  MissionManager mission_manager;
//  mission_manager.SetCommands(mission);
//  mission_manager.SetObstacles(obstacles);

//  mission_manager.Preprocess({0, 0, 10});
//  mission_manager.DumpMission();

//  for(int i = 0;i < 10;i++) {
//    ASSERT_TRUE(mission_manager.GetCurrentCommand().has_nothingcommand());
//    mission_manager.PopCommand();
//  }

//  for(int i = 0;i < 3;i++) {
//    ASSERT_TRUE(mission_manager.GetCurrentCommand().has_gotorawcommand());
//    mission_manager.PopCommand();
//  }

//  // Flush out the rest of the goto raw commands.
//  for(int i = 0;i < 100;i++) {
//    if(!mission_manager.GetCurrentCommand().has_gotorawcommand()) break;
//    mission_manager.PopCommand();
//  }

//  ASSERT_TRUE(mission_manager.GetCurrentCommand().has_sleepcommand());
//  mission_manager.PopCommand();

//  for(int i = 0;i < 3;i++) {
//    ASSERT_TRUE(mission_manager.GetCurrentCommand().has_gotorawcommand());
//    mission_manager.PopCommand();
//  }

//  // Flush out the rest of the goto raw commands.
//  for(int i = 0;i < 100;i++) {
//    if(!mission_manager.GetCurrentCommand().has_gotorawcommand()) break;
//    mission_manager.PopCommand();
//  }

//  ASSERT_TRUE(mission_manager.GetCurrentCommand().has_sleepcommand());
//  mission_manager.PopCommand();

//  for(int i = 0;i < 1000;i++) {
//    ASSERT_TRUE(mission_manager.GetCurrentCommand().has_nothingcommand());
//    mission_manager.PopCommand();
//  }
//}

} // namespace testing
} // namespace lib
