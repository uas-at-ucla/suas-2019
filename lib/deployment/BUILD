package(default_visibility = ['//visibility:public'])

# cc_test(
#   name = 'deployment_test',
#   srcs = [
#     'deployment_test.cc',
#   ],
#   deps = [
#     ':deployment',
#     '@gtest//:gtest_main',
#   ],
# )

cc_library(
  name = 'deployment',
  visibility = ['//visibility:public'],
  srcs = [
    'deployment.cc',
  ],
  hdrs = [
    'deployment.h',
  ],
  deps = [
    '//lib/ros:ros',
  ]
)
