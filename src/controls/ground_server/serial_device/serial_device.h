#pragma once

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <deque>
#include <functional>
#include <iostream>
#include <locale>
#include <mutex>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <google/protobuf/message.h>

// Based off code from https://gist.github.com/zguangyu/c93f28c952e8acc710c1
// Modified for use in transferring protobuf messages over serial devices.

// TODO(comran): Error checking.

namespace src {
namespace controls {
namespace ground_server {
namespace {
const int kSerialPortReadBuffer = 100;
const char kNewLineCharacter = '\n';
const int kSerialReadSleepPeriod = 50; // milliseconds
} // namespace

static inline ::std::string &ltrim(::std::string &s) {
  s.erase(
      s.begin(),
      ::std::find_if(s.begin(), s.end(),
                     ::std::not1(::std::ptr_fun<int, int>(::std::isspace))));
  return s;
}

static inline ::std::string &rtrim(::std::string &s) {
  s.erase(::std::find_if(s.rbegin(), s.rend(),
                         ::std::not1(::std::ptr_fun<int, int>(::std::isspace)))
              .base(),
          s.end());
  return s;
}

static inline std::string &trim(::std::string &s) { return ltrim(rtrim(s)); }

template <typename T> class SerialDevice {
 public:
  SerialDevice(::std::string portname, int speed, int parity) :
      messages_(0),
      read_thread_(&SerialDevice::ReadThread, this) {

    // Make sure that the template class is based off a protobuf.
    static_assert(::std::is_base_of<::google::protobuf::Message, T>::value,
                  "Template must be derived from a protobuf");

    // Open serial device for reading.
    fd_ = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (fd_ < 0) {
      fprintf(stderr, "Error opening serial port %s\n", portname.c_str());
      return;
    }

    // Create serial device TTY options structure, initialized with all 0s.
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd_, &tty) != 0) {
      fprintf(stderr, "Error %d from tcgetattr: %s\n", errno, strerror(errno));
      return;
    }

    // Set RX/TX baud rates.
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Terminal IO (termios) flags. Descriptions available here:
    // http://man7.org/linux/man-pages/man3/termios.3.html
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    // Apply TTY options.
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      fprintf(stderr, "Error %d from tcsetattr: %s\n", errno, strerror(errno));
    }
  }

  void ReadThread() {
    while (true) {
      // Continuously poll the serial device to check for new data.
      ReadPort();
      ::std::this_thread::sleep_for(
          ::std::chrono::milliseconds(kSerialReadSleepPeriod));
    }
  }

  void ReadPort() {
    // Check whether the serial device file descriptor was created successfully.
    if (fd_ < 0) {
      return;
    }

    // Create a buffer for reading in data over serial.
    ::std::vector<char> v(kSerialPortReadBuffer);

    // Read in data from serial device.
    int n = read(fd_, &v[0], kSerialPortReadBuffer);

    // Resize read vector to contain all data in the buffer.
    v.resize(n);

    // Move all characters to fifo queue for later processing. Also, count how
    // many messages we have received.
    {
      ::std::lock_guard<::std::mutex> lock(fifo_mutex_);

      for (auto i : v) {
        fifo_.push_back(i);

        if (i == kNewLineCharacter) {
          messages_++;
        }
      }
    }
  }

  void WritePort(::std::vector<char> &v) {
    // Check whether the serial device file descriptor was created successfully.
    if (fd_ < 0) {
      return;
    }

    // Attempt to write the data.
    int n = write(fd_, &v[0], v.size());

    // Report any writing errors that may have occurred.
    if (n < 0) {
      ::std::cerr << "Write error" << ::std::endl;
    }
  }

  T GetMessage() {
    // If there are no messages, don't process any.
    if (messages_ <= 0) {
      return "";
    }

    ::std::deque<char>::iterator iter = fifo_.begin();
    while (*iter != kNewLineCharacter) {
      iter++;
    }

    ::std::string s(fifo_.begin(), iter);

    iter++;

    {
      ::std::lock_guard<::std::mutex> lock(fifo_mutex_);
      fifo_.erase(fifo_.begin(), iter);
      messages_--;
    }

    ::std::cout << '"' << trim(s) << '"' << ::std::endl;

    return s;
  }

 private:
  int fd_;
  int messages_;
  ::std::mutex fifo_mutex_;
  ::std::deque<char> fifo_;
  ::std::thread read_thread_;
};

} // namespace ground_server
} // namespace controls
} // namespace src
