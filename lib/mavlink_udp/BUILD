cc_library(
  name = 'mavlink_udp',
  visibility = ['//visibility:public'],
  srcs = [
    'mavlink_udp.cc',
  ],
  hdrs = [
    'mavlink_udp.h',
  ],
  copts = [
    '-Wno-unused-parameter',
  ],
  deps = [
    "@cppzmq//:cppzmq",
    '@mavlink_v1//:mavlink_v1'
  ],
)
