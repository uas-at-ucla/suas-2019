proto_library(
  name = "serial_comms_message",
  srcs = [
    "serial_comms_message.proto",
  ],
  deps = [
    "@com_google_protobuf//:any_proto",
  ],
)

cc_proto_library(
  name = "serial_comms_message_cpp",
  deps = [
    ":serial_comms_message",
  ],
)


cc_library(
  name = 'serial_comms_bridge',
  visibility = ['//visibility:public'],
  srcs = [
    'serial_comms_bridge.cc',
  ],
  hdrs = [
    'serial_comms_bridge.h',
  ],
  deps = [
    ":serial_comms_message_cpp",

    "@cppzmq//:cppzmq",
    "@com_google_protobuf//:protobuf",
  ],
)
