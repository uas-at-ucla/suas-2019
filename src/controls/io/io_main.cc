#include "io.h"

int main(int argc, char **argv) {

  if (argc > 1) {
    ::ros::init(argc, argv, "uasatucla_io");
    ::src::controls::io::IO io(argv[1]);
    io.Run();
  } else {
    ::std::cerr << "Must pass in Pixhawk interface as an argument."
                << ::std::endl;
  }
}
