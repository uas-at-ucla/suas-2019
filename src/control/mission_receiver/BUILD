proto_library(
  name = "mission_commands",
  srcs = [
    "mission_commands.proto",
  ],
  deps = [
    "@com_google_protobuf//:any_proto",
  ],
)

cc_proto_library(
  name = "mission_commands_cpp",
  deps = [
    ":mission_commands",
  ],
)

cc_binary(
  name = 'mission_receiver',
  visibility = ['//visibility:public'],
  srcs = [
    'mission_receiver_main.cc'
  ],
  deps = [
    ":mission_receiver_lib"
  ],
)

cc_library(
  name = 'mission_receiver_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'mission_receiver.cc'
  ],
  hdrs = [
    'mission_receiver.h'
  ],
  deps = [
    ":mission_commands_cpp",
    '//src/control/loops:flight_loop_lib',
    '//aos/linux_code:init',
    "@socketio_client_cpp//:socketio_client_cpp",
    "@cppzmq//:cppzmq",
  ],
)
