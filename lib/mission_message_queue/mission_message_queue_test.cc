#include "mission_message_queue.h"

#include "gtest/gtest.h"

#include "lib/mission_manager/mission_manager.h"
#include "lib/mission_message_queue/mission_commands.pb.h"

namespace lib {
namespace testing {

TEST(FactorialTest, SendMissionOverQueueTest) {
  ::lib::mission_message_queue::MissionMessageQueueSender
      mission_message_queue_sender;

  ::lib::MissionManager mission_manager;
  ::lib::mission_message_queue::MissionMessageQueueReceiver
      mission_message_queue_receiver(&mission_manager);
}

}  // namespace testing
}  // namespace lib
