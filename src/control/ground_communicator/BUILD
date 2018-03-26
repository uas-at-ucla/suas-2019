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
  name = 'ground_communicator',
  visibility = ['//visibility:public'],
  srcs = [
    'ground_communicator_main.cc'
  ],
  deps = [
    ":ground_communicator_lib"
  ],
)

cc_library(
  name = 'ground_communicator_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'ground_communicator.cc'
  ],
  hdrs = [
    'ground_communicator.h'
  ],
  deps = [
    ":mission_commands_cpp",
    '//src/control/loops:flight_loop_lib',
    '//aos/linux_code:init',
    "@socketio_client_cpp//:socketio_client_cpp",
    "@cppzmq//:cppzmq",
  ],
)
