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
    '//lib/mavlink_serial:mavlink_serial'
  ],
  copts = [
    '-Wno-unused-parameter',
    '-Wno-missing-field-initializers',
  ],
)
