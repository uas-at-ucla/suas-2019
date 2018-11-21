#include <cassert>
#include <set>

#include "interface.h"
#include "udp.h"

namespace mavconn {
#define PFX "mavconn: "

// static members
::std::once_flag MAVConnInterface::init_flag;
::std::unordered_map<msgid_t, const mavlink_msg_entry_t *>
    MAVConnInterface::message_entries{};
::std::atomic<size_t> MAVConnInterface::conn_id_counter{0};

MAVConnInterface::MAVConnInterface(uint8_t system_id, uint8_t component_id) :
    sys_id(system_id),
    comp_id(component_id),
    m_status{},
    m_buffer{},
    tx_total_bytes(0),
    rx_total_bytes(0),
    last_tx_total_bytes(0),
    last_rx_total_bytes(0),
    last_iostat(steady_clock::now()) {
  conn_id = conn_id_counter.fetch_add(1);
  //::std::call_once(init_flag, init_msg_entry);
}

mavlink_status_t MAVConnInterface::get_status() { return m_status; }

MAVConnInterface::IOStat MAVConnInterface::get_iostat() {
  ::std::lock_guard<::std::recursive_mutex> lock(iostat_mutex);
  IOStat stat;

  stat.tx_total_bytes = tx_total_bytes;
  stat.rx_total_bytes = rx_total_bytes;

  auto d_tx = stat.tx_total_bytes - last_tx_total_bytes;
  auto d_rx = stat.rx_total_bytes - last_rx_total_bytes;
  last_tx_total_bytes = stat.tx_total_bytes;
  last_rx_total_bytes = stat.rx_total_bytes;

  auto now = steady_clock::now();
  auto dt = now - last_iostat;
  last_iostat = now;

  float dt_s = ::std::chrono::duration_cast<::std::chrono::seconds>(dt).count();

  stat.tx_speed = d_tx / dt_s;
  stat.rx_speed = d_rx / dt_s;

  return stat;
}

void MAVConnInterface::iostat_tx_add(size_t bytes) { tx_total_bytes += bytes; }

void MAVConnInterface::iostat_rx_add(size_t bytes) { rx_total_bytes += bytes; }

void MAVConnInterface::parse_buffer(const char *pfx, uint8_t *buf,
                                    const size_t bufsize,
                                    size_t bytes_received) {
  mavlink_status_t status;
  mavlink_message_t message;

  assert(bufsize >= bytes_received);

  iostat_rx_add(bytes_received);
  for (; bytes_received > 0; bytes_received--) {
    auto c = *buf++;

    // based on mavlink_parse_char()
    auto msg_received = static_cast<Framing>(
        mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
    if (msg_received == Framing::bad_crc ||
        msg_received == Framing::bad_signature) {
      _mav_parse_error(&m_status);
      m_status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      m_status.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        m_status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        m_buffer.len = 0;
        mavlink_start_checksum(&m_buffer);
      }
    }

    if (msg_received != Framing::incomplete) {
      log_recv(pfx, message, msg_received);

      if (message_received_cb)
        message_received_cb(&message, msg_received);
    }
  }
}

void MAVConnInterface::log_recv(const char *pfx, mavlink_message_t &msg,
                                Framing framing) {
  const char *framing_str =
      (framing == Framing::ok)
          ? "OK"
          : (framing == Framing::bad_crc)
                ? "!CRC"
                : (framing == Framing::bad_signature) ? "!SIG" : "ERR";

  const char *proto_version_str = (msg.magic == MAVLINK_STX) ? "v2.0" : "v1.0";

  // logDebug("%s%zu: recv: %s %4s Message-Id: %u [%u bytes] IDs: %u.%u Seq:
  // %u",
  //         pfx, conn_id, proto_version_str, framing_str, msg.msgid, msg.len,
  //         msg.sysid, msg.compid, msg.seq);
}

void MAVConnInterface::log_send(const char *pfx, const mavlink_message_t *msg) {
  const char *proto_version_str = (msg->magic == MAVLINK_STX) ? "v2.0" : "v1.0";

  // logDebug("%s%zu: send: %s Message-Id: %u [%u bytes] IDs: %u.%u Seq: %u",
  // pfx,
  //         conn_id, proto_version_str, msg->msgid, msg->len, msg->sysid,
  //         msg->compid, msg->seq);
}

void MAVConnInterface::send_message_ignore_drop(const mavlink_message_t *msg) {
  try {
    send_message(msg);
  } catch (::std::length_error &e) {
    //  logError(PFX "%zu: DROPPED Message-Id %u [%u bytes] IDs: %u.%u Seq: %u:
    //  %s",
    //           conn_id, msg->msgid, msg->len, msg->sysid, msg->compid,
    //           msg->seq,
    //           e.what());
  }
}

void MAVConnInterface::set_protocol_version(Protocol pver) {
  if (pver == Protocol::V10)
    m_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  else
    m_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
}

Protocol MAVConnInterface::get_protocol_version() {
  if (m_status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    return Protocol::V10;
  else
    return Protocol::V20;
}

/**
 * Parse host:port pairs
 */
static void url_parse_host(::std::string host, ::std::string &host_out,
                           int &port_out, const ::std::string def_host,
                           const int def_port) {
  ::std::string port;

  auto sep_it = ::std::find(host.begin(), host.end(), ':');
  if (sep_it == host.end()) {
    // host
    if (!host.empty()) {
      host_out = host;
      port_out = def_port;
    } else {
      host_out = def_host;
      port_out = def_port;
    }
    return;
  }

  if (sep_it == host.begin()) {
    // :port
    host_out = def_host;
  } else {
    // host:port
    host_out.assign(host.begin(), sep_it);
  }

  port.assign(sep_it + 1, host.end());
  port_out = ::std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 */
static void url_parse_query(::std::string query, uint8_t &sysid,
                            uint8_t &compid) {
  const ::std::string ids_end("ids=");
  ::std::string sys, comp;

  if (query.empty())
    return;

  auto ids_it =
      ::std::search(query.begin(), query.end(), ids_end.begin(), ids_end.end());
  if (ids_it == query.end()) {
    //  logWarn(PFX "URL: unknown query arguments");
    return;
  }

  ::std::advance(ids_it, ids_end.length());
  auto comma_it = ::std::find(ids_it, query.end(), ',');
  if (comma_it == query.end()) {
    //  logError(PFX "URL: no comma in ids= query");
    return;
  }

  sys.assign(ids_it, comma_it);
  comp.assign(comma_it + 1, query.end());

  sysid = ::std::stoi(sys);
  compid = ::std::stoi(comp);

  // logDebug(PFX "URL: found system/component id = [%u, %u]", sysid, compid);
}

static MAVConnInterface::Ptr url_parse_udp(::std::string hosts,
                                           ::std::string query,
                                           uint8_t system_id,
                                           uint8_t component_id, bool is_udpb) {
  ::std::string bind_pair, remote_pair;
  ::std::string bind_host, remote_host;
  int bind_port, remote_port;

  auto sep_it = ::std::find(hosts.begin(), hosts.end(), '@');
  if (sep_it == hosts.end()) {
    //  logError(PFX "UDP URL should contain @!");
    throw DeviceError("url", "UDP separator not found");
  }

  bind_pair.assign(hosts.begin(), sep_it);
  remote_pair.assign(sep_it + 1, hosts.end());

  // udp://0.0.0.0:14555@:14550
  url_parse_host(bind_pair, bind_host, bind_port, "0.0.0.0",
                 MAVConnUDP::DEFAULT_BIND_PORT);
  url_parse_host(remote_pair, remote_host, remote_port,
                 MAVConnUDP::DEFAULT_REMOTE_HOST,
                 MAVConnUDP::DEFAULT_REMOTE_PORT);
  url_parse_query(query, system_id, component_id);

  if (is_udpb)
    remote_host = MAVConnUDP::BROADCAST_REMOTE_HOST;

  return ::std::make_shared<MAVConnUDP>(system_id, component_id, bind_host,
                                        bind_port, remote_host, remote_port);
}

MAVConnInterface::Ptr MAVConnInterface::open_url(::std::string url,
                                                 uint8_t system_id,
                                                 uint8_t component_id) {
  const ::std::string proto_end("://");
  ::std::string proto;
  ::std::string host;
  ::std::string path;
  ::std::string query;

  auto proto_it =
      ::std::search(url.begin(), url.end(), proto_end.begin(), proto_end.end());

  // copy protocol
  proto.reserve(::std::distance(url.begin(), proto_it));
  ::std::transform(url.begin(), proto_it, ::std::back_inserter(proto),
                   ::std::ref(tolower));

  // copy host
  ::std::advance(proto_it, proto_end.length());
  auto path_it = ::std::find(proto_it, url.end(), '/');
  ::std::transform(proto_it, path_it, ::std::back_inserter(host),
                   ::std::ref(tolower));

  // copy path, and query if exists
  auto query_it = ::std::find(path_it, url.end(), '?');
  path.assign(path_it, query_it);
  if (query_it != url.end())
    ++query_it;
  query.assign(query_it, url.end());

  // logDebug(PFX "URL: %s: proto: %s, host: %s, path: %s, query: %s",
  // url.c_str(),
  //         proto.c_str(), host.c_str(), path.c_str(), query.c_str());

  if (proto == "udp")
    return url_parse_udp(host, query, system_id, component_id, false);
  else if (proto == "udp-b")
    return url_parse_udp(host, query, system_id, component_id, true);
  else
    throw DeviceError("url", "Unknown URL type");
}
} // namespace mavconn
