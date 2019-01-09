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
    '//src/control/loops:flight_loop_lib',
    '//src/control/io:io_lib',
    '//src/control:messages_cpp',
    '//lib/mission_message_queue:mission_message_queue',
    '//lib/logger:log_sender',
    '//lib/serial_comms:serial_comms_bridge',
    '//lib/phased_loop:phased_loop',
    '//lib/proto_comms:proto_comms',
    '//lib/base64_tools:base64_tools',
    "@socketio_client_cpp//:socketio_client_cpp",
    "@cppzmq//:cppzmq",
  ],
)
