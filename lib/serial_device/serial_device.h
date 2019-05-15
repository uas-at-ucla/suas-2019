#pragma once

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <locale>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <boost/crc.hpp>
#include <google/protobuf/message.h>

#include "lib/base64_tools/base64_tools.h"

// Based off code from https://gist.github.com/zguangyu/c93f28c952e8acc710c1
// Modified for use in transferring protobuf messages over serial devices.

// TODO(comran): Error checking.

namespace lib {
namespace serial_device {
namespace {
static const int kSerialPortReadBuffer = 100;
static const int kSerialReadSleepPeriod = 50; // milliseconds
static const int kCrcLength = 8;

static const ::std::string kMessagePreamble = "UAS@UCLA_";
static const char kCarriageReturn = '\r';
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

  void Quit() {
    run_ = false;
    close(fd_);
    // read_thread_.join();
  }

  void ReadThread() {
    while (run_) {
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

    if (n < 0) {
      return;
    }

    // Resize read vector to contain all data in the buffer.
    v.resize(n);

    // Move all characters to fifo queue for later processing. Also, count how
    // many messages we have received.

    for (char i : v) {
      if (i == '\n') {
        RetrieveDecodedMessage(buffer_);
        buffer_ = "";
      } else {
        buffer_ += i;
      }
    }
  }

  void WritePort(::std::string data) {
    // Generate a message line using a custom encoding scheme.
    ::std::string encoded_message = GenerateEncodedMessage(data);

    // ::std::cout << "writing " << encoded_message << ::std::endl;

    // Put string data into a vector.
    ::std::vector<char> v(encoded_message.begin(), encoded_message.end());

    // Check whether the serial device file descriptor was created successfully.
    if (fd_ < 0) {
      return;
    }

    // Attempt to write the data to the serial device.
    int n = write(fd_, &v[0], v.size());

    // Report any writing errors that may have occurred.
    if (n < 0) {
      ::std::cerr << "Write error" << ::std::endl;
    }
  }

  void WritePort(T proto_message) {
    // Serialize the given protobuf to a string for sending over serial.
    ::std::string proto_string;
    proto_message.SerializeToString(&proto_string);

    // Encode the protobuf message in base64 to eliminate any weird symbols.
    proto_string = ::lib::base64_tools::Encode(proto_string);

    // Write string output;
    WritePort(proto_string);
  }

  bool GetLatestProto(T &proto_dest) {
    ::std::lock_guard<::std::mutex> lock(proto_queue_mutex_);

    // Check whether there is a proto available to fetch. If not, return false
    // to indicate that proto_dest was not changed.
    if (proto_queue_.empty()) {
      return false;
    }

    // Copy the top value off the queue.
    proto_dest.CopyFrom(proto_queue_.front());

    // Empty the proto queue.
    while (!proto_queue_.empty()) {
      proto_queue_.pop();
    }

    return true;
  }

 private:
  unsigned CalculateCrc(::std::string data) {
    ::boost::crc_32_type crc;
    crc.process_bytes(data.c_str(), data.length());

    return crc.checksum();
  }

  ::std::string GenerateEncodedMessage(::std::string data) {
    // Remove all carriage returns from data. This character is reserved for
    // indicating a new line was received over the serial connection.
    data.erase(::std::remove(data.begin(), data.end(), kCarriageReturn),
               data.end());

    // Calculate CRC for the string data.
    unsigned crc = CalculateCrc(data);

    ::std::stringstream write_data_stream;
    write_data_stream << kMessagePreamble;
    write_data_stream << ::std::hex << ::std::setfill('0') << ::std::setw(8)
                      << crc;
    write_data_stream << data;
    write_data_stream << kCarriageReturn;

    return write_data_stream.str();
  }

  void RetrieveDecodedMessage(::std::string buffer) {
    size_t message_start = buffer.find(kMessagePreamble);

    // Check if message not found.
    if (message_start == ::std::string::npos) {
      return;
    }

    // Check if 8 character CRC exists.
    if (buffer.length() <
        message_start + kMessagePreamble.length() + kCrcLength) {
      return;
    }

    ::std::string crc = buffer.substr(kMessagePreamble.length(), kCrcLength);

    // Extract data and decode from base64.
    ::std::string data =
        buffer.substr(kMessagePreamble.length() + kCrcLength,
                      buffer.length() - kMessagePreamble.length() - kCrcLength);
    data = ::lib::base64_tools::Decode(data);

    T proto_message;
    if (proto_message.ParseFromString(data)) {
      ::std::lock_guard<::std::mutex> lock(proto_queue_mutex_);
      while (!proto_queue_.empty()) {
        proto_queue_.pop();
      }

      proto_queue_.push(proto_message);
    }
  }

  int fd_;
  int messages_;
  ::std::string buffer_;
  ::std::thread read_thread_;
  ::std::queue<T> proto_queue_;
  ::std::mutex proto_queue_mutex_;

  ::std::atomic<bool> run_{true};
};

} // namespace serial_device
} // namespace lib
