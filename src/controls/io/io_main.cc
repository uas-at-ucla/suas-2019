#include "io.h"

int main(int argc, char **argv) {
  if (argc > 1) {
    ::src::controls::io::IO io(argv[1]);
    io.Run();
  }
}
