cc_library(
  name = 'mavros',
  visibility = ['//visibility:public'],
  srcs = glob([
    'mavros/src/**/*.cpp',
  ]),
  hdrs = glob([
    'mavros/include/**/*.h',
  ]),
  includes = [
    'mavros/include',
  ],
  deps = [
    '@boost//:asio',
    '@eigen//:eigen',
  ],
)

cc_library(
  name = 'libmavconn',
  visibility = ['//visibility:public'],
  srcs = glob([
    'libmavconn/src/**/*.cpp',
  ]),
  hdrs = glob([
    'libmavconn/include/**/*.h',
  ]),
  includes = [
    'libmavconn/include/mavconn'
  ],
  deps = [
    ':mavros',
    '@boost//:asio',
  ],
)

cc_test(
  name = "libmavconn_test",
  srcs = [
    "libmavconn/test/test_mavconn.cpp"
  ],
  deps = [
    ":libmavconn",
    "@gtest//:googletest_main",
  ],
)
