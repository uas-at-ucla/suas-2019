cc_library(
  name = 'cppzmq',
  visibility = ["//visibility:public"],
  hdrs = glob([
    'zmq.hpp',
    'zmq_addon.hpp',
  ]),
  deps = [
    '@libzmq//:libzmq',
  ],
  copts = [
    '-DZMQ_BUILD_DRAFT_API=1',
  ],
)
