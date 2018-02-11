cc_binary(
  name = 'io',
  visibility = ['//visibility:public'],
  srcs = [
    'io_main.cc'
  ],
  deps = [
    ":io_lib"
  ],
)

cc_library(
  name = 'io_lib',
  visibility = ['//visibility:public'],
  srcs = [
    'io.cc'
  ],
  hdrs = [
    'io.h'
  ],
  deps = [
    "//src/control/io/autopilot_interface:autopilot_interface"
  ],
)
