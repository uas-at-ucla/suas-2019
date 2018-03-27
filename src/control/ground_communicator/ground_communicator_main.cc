#include "ground_communicator.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::InitNRT();
  ::src::control::ground_communicator::MissionReceiver ground_communicator;
  ground_communicator.Run();
  ::aos::Cleanup();
}
