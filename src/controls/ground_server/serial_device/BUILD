cc_library(
  name = 'serial_device',
  visibility = ['//visibility:public'],
  hdrs = [
    'serial_device.h'
  ],
  deps = [
    '//src/controls:messages_cpp',
    "@com_google_protobuf//:protobuf",
  ],
)

cc_test(
  name = 'serial_device_test',
  srcs = [
    'serial_device_test.cc',
  ],
  deps = [
    ':serial_device',
    '//src/controls:messages_cpp',
    '@gtest//:gtest_main',
  ],
)
