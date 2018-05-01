cc_library(
  name = 'autopilot_interface',
  visibility = ['//visibility:public'],
  srcs = [
    'autopilot_interface.cc',
  ],
  hdrs = [
    'autopilot_interface.h',
  ],
  deps = [
    '//lib/mavlink_udp:mavlink_udp'
  ],
  copts = [
    '-Wno-unused-parameter',
    '-Wno-missing-field-initializers',
  ],
)
