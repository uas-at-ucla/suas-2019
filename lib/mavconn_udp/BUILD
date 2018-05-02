cc_library(
  name = 'mavconn_udp',
  visibility = ['//visibility:public'],
  srcs = [
    'udp.cc',
    'interface.cc',
  ],
  hdrs = [
    'udp.h',
    'interface.h',
    'msgbuffer.h',
  ],
  copts = [
    '-Wno-unused-parameter',
    '-Wno-unused-value',
    '-Wno-reorder',
    '-Wno-unused-variable',
    '-Wno-sign-compare',
    '-Wno-missing-field-initializers',
  ],
  deps = [
    "@mavlink_v2//:mavlink_v2",
    "@boost//:asio",
    "@console_bridge//:console_bridge",
  ],
)
