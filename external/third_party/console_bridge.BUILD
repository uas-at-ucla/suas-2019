cc_library(
  name = 'console_bridge_genfiles',
  visibility = ['//visibility:public'],
  hdrs = [
    'console_bridge_export.h',
  ],
)

cc_library(
  name = 'console_bridge',
  visibility = ['//visibility:public'],
  srcs = glob([
    'src/**/*.cpp',
  ]),
  hdrs = glob([
    'include/**/*.h',
  ]),
  includes = [
    'include',
  ],
  deps = [
    '@boost//:asio',
    '@eigen//:eigen',
    ':console_bridge_genfiles',
  ],
  copts = [
    "-Wno-format-nonliteral",
  ],
)

cc_test(
  name = "console_bridge_test",
  srcs = [
    "test/console_TEST.cc"
  ],
  deps = [
    ":console_bridge",
    "@gtest//:googletest_main",
  ],
)
