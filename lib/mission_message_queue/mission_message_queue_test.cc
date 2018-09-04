#include "mission_message_queue.h"

#include <thread>
#include <unistd.h>

#include "gtest/gtest.h"

#include "lib/mission_manager/mission_manager.h"

namespace lib {
namespace testing {

// TODO(comran): Fix this test.
// TEST(MissionMessageQueueTest, SendMissionOverQueueTest) {
//  ::lib::mission_message_queue::MissionMessageQueueSender
//      mission_message_queue_sender;

//  ::lib::mission_message_queue::MissionMessageQueueReceiver
//      mission_message_queue_receiver;

//  usleep(1e6);

//  ::lib::mission_manager::GroundData ground_data;
//  ::lib::mission_manager::Mission *mission =
//      new ::lib::mission_manager::Mission();

//  for (int i = 0; i < 100; i++) {
//    ::lib::mission_manager::Command *cmd = mission->add_commands();

//    mission_manager::GotoCommand *goto_cmd = cmd->mutable_gotocommand();
//    mission_manager::Position3D *goal = goto_cmd->mutable_goal();
//    goal->set_latitude(1.0);
//    goal->set_longitude(3.0);
//    goal->set_altitude(3.0);
//    goto_cmd->set_come_to_stop(false);
//  }

//  ground_data.set_allocated_mission(mission);
//  mission_message_queue_sender.SendData(ground_data);

//  usleep(1e4);

//  ASSERT_EQ(
//      mission_message_queue_receiver.get_mission_manager()->NumberOfCommands(),
//      100);
//}

} // namespace testing
} // namespace lib
