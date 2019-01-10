package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
  name = "ros_common",
  srcs = glob([
    "lib/amd64/**/*.so",
  ]),
  hdrs = glob([
    "include/**/*.h",
  ]),
  includes = ["include"],
  deps = [
    "@boost//:math",
    "@boost//:call_traits",
    "@boost//:lexical_cast",
  ],
)
