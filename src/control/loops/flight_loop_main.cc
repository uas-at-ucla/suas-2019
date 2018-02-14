#include "flight_loop.h"

#include "aos/linux_code/init.h"

int main() {
  ::aos::InitNRT();
  ::spinny::control::loops::FlightLoop flight_loop;
  flight_loop.Run();
  ::aos::Cleanup();
}
