cc_library(
  name = 'spdlog',
  visibility = ["//visibility:public"],
  srcs = glob([
    'include/**/*.cc',
  ]),
  hdrs = glob([
    'include/**/*.h',
    'include/spdlog/fmt/bundled/format.cc',
  ]),
  includes = [
    'include',
  ],
  copts = [
    '-Wno-format-nonliteral',
  ],
)
