package(default_visibility = ['//visibility:public'])

load('//aos/build:queues.bzl', 'queue_library')

queue_library(
  name = 'flight_loop_queue',
  srcs = [
    'flight_loop.q',
  ],
)

cc_binary(
  name = 'flight_loop',
  visibility = ['//visibility:public'],
  srcs = [
    'flight_loop_main.cc'
  ],
  deps = [
    ":flight_loop_lib"
  ],
)

cc_library(
  name = 'flight_loop_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'flight_loop.cc'
  ],
  hdrs = [
    'flight_loop.h'
  ],
  deps = [
    ':flight_loop_queue',
    '//aos/common/util:phased_loop',
    '//aos/linux_code:init',
    '//aos/common:time',
  ],
)
