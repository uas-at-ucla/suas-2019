#include "proto_comms.h"

#include <thread>
#include <unistd.h>

#include "gtest/gtest.h"

namespace lib {
namespace proto_comms {
namespace testing {

TEST(ProtoCommsTest, SendReceiveTest) {
  ProtoSender tx("ipc:///tmp/uasatucla_proto_comms_test.ipc");
  ProtoReceiver rx("ipc:///tmp/uasatucla_proto_comms_test.ipc", 10);

  usleep(1e6);

  ::zmq::message_t message_zmq(4);
  memcpy((void *)message_zmq.data(), "test", 4);

  tx.Send(&message_zmq);
  usleep(1e4);
  ::std::string latest = rx.GetLatest();

  ASSERT_STREQ("test", latest.c_str());
}

TEST(ProtoCommsTest, SendReceiveMultipleTest) {
  ProtoSender tx("ipc:///tmp/uasatucla_proto_comms_test.ipc");
  ProtoReceiver rx("ipc:///tmp/uasatucla_proto_comms_test.ipc", 10);

  usleep(1e6);

  for(int i = 0;i < 100;i++) {
    std::ostringstream s;
    s << "TEST" << i;

    ::zmq::message_t message_zmq(s.str().size());
    memcpy((void *)message_zmq.data(), s.str().c_str(), s.str().size());

    tx.Send(&message_zmq);
  }

  usleep(1e4);

  ::std::string latest = rx.GetLatest();
  ::std::queue<::std::string> queue = rx.GetQueue();

  ASSERT_STREQ("TEST99", latest.c_str());
  ASSERT_EQ(queue.size(), 10);

  for(int i = 90;i < 99;i++) {
    std::ostringstream s;
    s << "TEST" << i;

    ASSERT_STREQ(s.str().c_str(), queue.front().c_str());

    queue.pop();
  }
}

} // namespace testing
} // namespace proto_comms
} // namespace lib
