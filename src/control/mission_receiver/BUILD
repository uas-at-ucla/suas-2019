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
    '//src/control/loops:flight_loop_queue',
    '//aos/linux_code:init',
    "@socketio_client_cpp//:socketio_client_cpp",
    "@cppzmq//:cppzmq",
  ],
)
