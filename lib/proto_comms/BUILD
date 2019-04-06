cc_library(
  name = 'proto_comms',
  visibility = ['//visibility:public'],
  srcs = [
    'proto_comms.cc',
  ],
  hdrs = [
    'proto_comms.h',
  ],
  deps = [
    "@cppzmq//:cppzmq",
    "@com_google_protobuf//:protobuf",
    "//lib/semaphore:semaphore",
  ],
)

cc_test(
  name = 'proto_comms_test',
  visibility = ['//visibility:public'],
  srcs = [
    'proto_comms_test.cc',
  ],
  deps = [
    ":proto_comms",
    "@gtest//:gtest_main",
  ],
)
