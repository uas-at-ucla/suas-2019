package(default_visibility = ['//visibility:public'])

proto_library(
  name = "log_message",
  srcs = [
    "log_message.proto",
  ],
  deps = [
    "@com_google_protobuf//:any_proto",
  ],
)

cc_proto_library(
  name = "log_message_cpp",
  deps = [
    ":log_message",
  ],
)

cc_binary(
  name = 'log_writer',
  visibility = ['//visibility:public'],
  srcs = [
    'log_writer_main.cc',
  ],
  deps = [
    ":log_writer_lib",
  ]
)

cc_library(
  name = 'log_writer_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'log_writer.cc',
  ],
  hdrs = [
    'log_writer.h',
  ],
  deps = [
    ":log_message_cpp",
    "@cppzmq//:cppzmq",
    "@com_google_protobuf//:protobuf",
    "@spdlog//:spdlog",
  ],
)

cc_library(
  name = 'log_sender',
  visibility = ['//visibility:public'],
  srcs = [
    'log_sender.cc',
  ],
  hdrs = [
    'log_sender.h',
  ],
  deps = [
    ":log_message_cpp",
    "@cppzmq//:cppzmq",
    "@com_google_protobuf//:protobuf",
  ],
)

cc_test(
  name = 'log_test',
  visibility = ['//visibility:public'],
  srcs = [
    'log_test.cc',
  ],
  deps = [
    ":log_sender",
    ":log_writer_lib",
    "@gtest//:googletest_main",
  ],
)
