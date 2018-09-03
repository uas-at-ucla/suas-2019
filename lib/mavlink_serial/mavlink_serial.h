#pragma once

#include <cstdlib>
#include <fcntl.h>   // File control definitions
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>
#include <stdio.h>   // Standard input/output definitions
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // UNIX standard function definitions
#include <vector>

#include <common/mavlink.h>

namespace lib {
namespace mavlink_serial {

// The following two non-standard baudrates should have been defined by the
// system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

// Status flags
#define SERIAL_PORT_OPEN 1;
#define SERIAL_PORT_CLOSED 0;
#define SERIAL_PORT_ERROR -1;

/* This object handles the opening and closing of the offboard computer's
 * serial port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * gaurds any port operation with a pthread mutex. */
class MavlinkSerial {
 public:
  MavlinkSerial();
  MavlinkSerial(const char *uart_name_, int baudrate_);
  void initialize_defaults();
  ~MavlinkSerial();

  bool debug;
  const char *uart_name;
  int baudrate;
  int status;

  int read_messages(::std::vector<mavlink_message_t> &message);
  int write_message(const mavlink_message_t &message);

  int open_serial();
  void close_serial();

  void start();
  void stop();

  void handle_quit(int sig);

 private:
  int fd;
  mavlink_status_t lastStatus;
  pthread_mutex_t lock;

  int _open_port(const char *port);
  bool _setup_port(int baud, int data_bits, int stop_bits, bool parity,
                   bool hardware_control);
  int _read_port(uint8_t *cp);
  int _write_port(char *buf, unsigned len);
};

} // namespace mavlink_serial
} // namespace lib
