#include "mission_message_queue.h"

#include <unistd.h>
#include <thread>

#include "gtest/gtest.h"

#include "lib/mission_manager/mission_manager.h"
#include "lib/mission_message_queue/mission_commands.pb.h"

namespace lib {
namespace testing {

TEST(MissionMessageQueueTest, SendMissionOverQueueTest) {
  for (int runs = 0; runs < 5; runs++) {
    ::std::cout << "run " << runs << ::std::endl;
    ::lib::mission_message_queue::MissionMessageQueueSender
        mission_message_queue_sender;

    ::lib::mission_message_queue::MissionMessageQueueReceiver
        mission_message_queue_receiver;

    usleep(1e6);

    for (int i = 0; i < 5; i++) {
      ::lib::mission_message_queue::Mission mission;
      ::lib::mission_message_queue::Command *cmd = mission.add_commands();
      cmd->set_type("goto");
      cmd->set_latitude(1.0);
      cmd->set_longitude(3.0);
      cmd->set_altitude(3.0);

      mission_message_queue_sender.SendMission(mission);
    }

    usleep(1e4);

    ASSERT_GT(mission_message_queue_receiver.get_mission_manager()
                  ->NumberOfCommands(),
              0);
  }
}

}  // namespace testing
}  // namespace lib
