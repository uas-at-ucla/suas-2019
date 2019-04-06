load('//tools/build_rules:select.bzl', 'cpu_select')

cc_library(
  name = 'ros',
  visibility = ['//visibility:public'],
  deps = [
    '@boost//:function',
  ] + cpu_select({
    'amd64': [
      '@ros_bazel//:ros_amd64',
    ],
    'raspi': [
      '@ros_bazel//:ros_raspi',
    ],
  }),
)