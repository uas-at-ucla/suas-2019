#include "flight_loop.h"

int main() {
  ::src::control::loops::FlightLoop flight_loop;
  flight_loop.Run();
}
