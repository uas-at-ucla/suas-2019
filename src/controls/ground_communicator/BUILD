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
    '//src/controls:messages_cpp',
    '//lib/ros:ros',
    "@socketio_client_cpp//:socketio_client_cpp",
  ],
)
