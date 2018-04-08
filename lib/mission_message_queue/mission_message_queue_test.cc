#include "mission_message_queue.h"

#include <unistd.h>
#include <thread>

#include "gtest/gtest.h"

#include "lib/mission_manager/mission_manager.h"

namespace lib {
namespace testing {

TEST(MissionMessageQueueTest, SendMissionOverQueueTest) {
  ::lib::mission_message_queue::MissionMessageQueueSender
      mission_message_queue_sender;

  ::lib::mission_message_queue::MissionMessageQueueReceiver
      mission_message_queue_receiver;

  usleep(1e6);

  ::lib::mission_manager::Mission mission;

  for (int i = 0; i < 100; i++) {
    ::lib::mission_manager::Command *cmd = mission.add_commands();

    mission_manager::GotoCommand *goto_cmd = cmd->mutable_gotocommand();
    goto_cmd->set_latitude(1.0);
    goto_cmd->set_longitude(3.0);
    goto_cmd->set_altitude(3.0);
  }

  mission_message_queue_sender.SendMission(mission);

  usleep(1e4);

  ASSERT_EQ(
      mission_message_queue_receiver.get_mission_manager()->NumberOfCommands(),
      100);
}

}  // namespace testing
}  // namespace lib
