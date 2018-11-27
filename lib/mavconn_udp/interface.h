#pragma once

#include <boost/system/system_error.hpp>

#include <atomic>
#include <cassert>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

#include <common/mavlink.h>

#include "msgbuffer.h"

namespace mavconn {
using steady_clock = std::chrono::steady_clock;
using lock_guard = std::lock_guard<std::recursive_mutex>;
using msgid_t = uint32_t;

//! Same as @p common::MAV_COMPONENT::COMP_ID_UDP_BRIDGE
static constexpr auto MAV_COMP_ID_UDP_BRIDGE = 240;

//! Rx packer framing status. (same as @p mavlink_framing_t)
enum class Framing : uint8_t {
  incomplete = MAVLINK_FRAMING_INCOMPLETE,
  ok = MAVLINK_FRAMING_OK,
  bad_crc = MAVLINK_FRAMING_BAD_CRC,
  bad_signature = MAVLINK_FRAMING_BAD_SIGNATURE,
};

//! MAVLink protocol version
enum class Protocol : uint8_t {
  V10 = 1, //!< MAVLink v1.0
  V20 = 2  //!< MAVLink v2.0
};

/**
 * @brief Common exception for communication error
 */
class DeviceError : public std::runtime_error {
 public:
  /**
   * @breif Construct error.
   */
  template <typename T>
  DeviceError(const char *module, T msg) :
      std::runtime_error(make_message(module, msg)) {}

  template <typename T>
  static std::string make_message(const char *module, T msg) {
    std::ostringstream ss;
    ss << "DeviceError:" << module << ":" << msg_to_string(msg);
    return ss.str();
  }

  static std::string msg_to_string(const char *description) {
    return description;
  }

  static std::string msg_to_string(int errnum) { return ::strerror(errnum); }

  static std::string msg_to_string(boost::system::system_error &err) {
    return err.what();
  }
};

/**
 * @brief Generic mavlink interface
 */
class MAVConnInterface {
 private:
  MAVConnInterface(const MAVConnInterface &) = delete;

 public:
  using ReceivedCb = std::function<void(const mavlink_message_t *message,
                                        const Framing framing)>;
  using ClosedCb = std::function<void(void)>;
  using Ptr = std::shared_ptr<MAVConnInterface>;
  using ConstPtr = std::shared_ptr<MAVConnInterface const>;
  using WeakPtr = std::weak_ptr<MAVConnInterface>;

  struct IOStat {
    size_t tx_total_bytes; //!< total bytes transferred
    size_t rx_total_bytes; //!< total bytes received
    float tx_speed;        //!< current transfer speed [B/s]
    float rx_speed;        //!< current receive speed [B/s]
  };

  /**
   * @param[in] system_id     sysid for send_message
   * @param[in] component_id  compid for send_message
   */
  MAVConnInterface(uint8_t system_id = 1,
                   uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE);

  /**
   * @brief Close connection.
   */
  virtual void close() = 0;

  /**
   * @brief Send message (mavlink_message_t)
   *
   * Can be used to forward messages from other connection channel.
   *
   * @note Does not do finalization!
   *
   * @throws std::length_error  On exceeding Tx queue limit (MAX_TXQ_SIZE)
   * @param[in] *message  not changed
   */
  virtual void send_message(const mavlink_message_t *message) = 0;

  /**
   * @brief Send raw bytes (for some quirks)
   * @throws std::length_error  On exceeding Tx queue limit (MAX_TXQ_SIZE)
   */
  virtual void send_bytes(const uint8_t *bytes, size_t length) = 0;

  /**
   * @brief Send message and ignore possible drop due to Tx queue limit
   */
  void send_message_ignore_drop(const mavlink_message_t *message);

  //! Message receive callback
  ReceivedCb message_received_cb;
  //! Port closed notification callback
  ClosedCb port_closed_cb;

  virtual mavlink_status_t get_status();
  virtual IOStat get_iostat();
  virtual bool is_open() = 0;

  inline uint8_t get_system_id() { return sys_id; }
  inline void set_system_id(uint8_t sysid) { sys_id = sysid; }
  inline uint8_t get_component_id() { return comp_id; }
  inline void set_component_id(uint8_t compid) { comp_id = compid; }

  /**
   * Set protocol used for encoding Mavlink messages.
   */
  void set_protocol_version(Protocol pver);
  Protocol get_protocol_version();

  /**
   * @brief Construct connection from URL
   *
   * Supported URL schemas:
   * - serial://
   * - udp://
   * - tcp://
   * - tcp-l://
   *
   * Please see user's documentation for details.
   *
   * @param[in] url           resource locator
   * @param[in] system_id     optional system id
   * @param[in] component_id  optional component id
   * @return @a Ptr to constructed interface class,
   *         or throw @a DeviceError if error occured.
   */
  static Ptr open_url(std::string url, uint8_t system_id = 1,
                      uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE);

  static std::vector<std::string> get_known_dialects();

 protected:
  uint8_t sys_id;  //!< Connection System Id
  uint8_t comp_id; //!< Connection Component Id

  //! Maximum mavlink packet size + some extra bytes for padding.
  static constexpr size_t MAX_PACKET_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
  //! Maximum count of transmission buffers.
  static constexpr size_t MAX_TXQ_SIZE = 1000;

  //! This map merge all dialect mavlink_msg_entry_t structs. Needed for packet
  //! parser.
  static std::unordered_map<msgid_t, const mavlink_msg_entry_t *>
      message_entries;

  //! Channel number used for logging.
  size_t conn_id;

  inline mavlink_status_t *get_status_p() { return &m_status; }

  inline mavlink_message_t *get_buffer_p() { return &m_buffer; }

  /**
   * Parse buffer and emit massage_received.
   */
  void parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize,
                    size_t bytes_received);

  void iostat_tx_add(size_t bytes);
  void iostat_rx_add(size_t bytes);

  void log_recv(const char *pfx, mavlink_message_t &msg, Framing framing);
  void log_send(const char *pfx, const mavlink_message_t *msg);

 private:
  friend const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid);

  mavlink_status_t m_status;
  mavlink_message_t m_buffer;

  std::atomic<size_t> tx_total_bytes, rx_total_bytes;
  std::recursive_mutex iostat_mutex;
  size_t last_tx_total_bytes, last_rx_total_bytes;
  std::chrono::time_point<steady_clock> last_iostat;

  //! monotonic counter (increment only)
  static std::atomic<size_t> conn_id_counter;

  //! init_msg_entry() once flag
  static std::once_flag init_flag;

  /**
   * Initialize message_entries map
   *
   * autogenerated. placed in mavlink_helpers.cpp
   */
  static void init_msg_entry();
};
} // namespace mavconn
