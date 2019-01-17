package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
  name = "ros_raspi",
  srcs = glob([
      "lib/raspi/**/*.so*",
  ]),
  hdrs = glob([
    "include/**/*.h",
  ]),
  includes = ["include"],
  deps = [
    "@boost//:math",
    "@boost//:call_traits",
    "@boost//:lexical_cast",
    "@boost//:filesystem",
    "@boost//:program_options",
    "@boost//:thread",
  ],
)

cc_library(
  name = "ros_amd64",
  srcs = glob([
      "lib/amd64/**/*.so*",
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
