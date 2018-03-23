#include "mission_receiver.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::InitNRT();
  ::src::control::mission_receiver::MissionReceiver mission_receiver;
  mission_receiver.Run();
  ::aos::Cleanup();
}
