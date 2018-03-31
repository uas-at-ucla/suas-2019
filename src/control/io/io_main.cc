#include "io.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::InitNRT();
  ::src::control::io::IO io;
  io.Run();
  ::aos::Cleanup();
}
