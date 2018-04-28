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
    '//lib/mission_message_queue:mission_message_queue',
    '//lib/logger:log_sender',
    '//aos/linux_code:init',
    "@socketio_client_cpp//:socketio_client_cpp",
    "@cppzmq//:cppzmq",
  ],
)
