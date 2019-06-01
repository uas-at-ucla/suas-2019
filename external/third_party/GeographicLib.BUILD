cc_library(
  name = 'GeographicLib',
  visibility = ["//visibility:public"],
  srcs = glob([
    'src/**/*.cpp',
  ]),
  hdrs = glob([
    'include/**/*.hpp',
    'include/**/*.h',
    'include/**/*.h',
  ]),
  includes = [
    'include',
  ],
  copts = [
    '-Wno-shift-negative-value'
  ],
)
