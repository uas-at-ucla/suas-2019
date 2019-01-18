package(default_visibility = ['//visibility:public'])

filegroup(
    name = "empty",
    srcs = [],
)

filegroup(
  name = "toolchain_fg",
  srcs = [
    ":cc-compiler-k8",
    ":cc-compiler-raspi",
    '@raspi-toolchain//:compiler_components',
  ],
)

cc_toolchain_suite(
  name = 'toolchain',
  # target_cpu | compiler
  toolchains = {
    'raspi': ':cc-compiler-raspi',
    'k8': ':cc-compiler-k8',
    'darwin|clang': ':cc-compiler-k8',
  },
)

cc_toolchain(
  name = "cc-compiler-k8",
  all_files = ":empty",
  compiler_files = ":empty",
  cpu = "local",
  dwp_files = ":empty",
  dynamic_runtime_libs = [":empty"],
  linker_files = ":empty",
  objcopy_files = ":empty",
  static_runtime_libs = [":empty"],
  strip_files = ":empty",
  supports_param_files = 1,
)

filegroup(
  name = 'raspi-gcc-files',
  srcs = [
    '//tools/cpp/raspi-toolchain:tool-wrappers',
    '@raspi-toolchain//:compiler_pieces',
  ],
)

filegroup(
  name = 'linaro_linux_linker_files',
  srcs = [
    '//tools/cpp/raspi-toolchain:gcc',
    '//tools/cpp/raspi-toolchain:ld',
    '//tools/cpp/raspi-toolchain:ar',
    '@raspi-toolchain//:compiler_pieces',
  ],
)

filegroup(
  name = 'linaro_linux_compiler_files',
  srcs = [
    '//tools/cpp/raspi-toolchain:gcc',
    '//tools/cpp/raspi-toolchain:ld',
    '//tools/cpp/raspi-toolchain:as',
  ],
)

cc_toolchain(
  name = 'cc-compiler-raspi',
  all_files = ':raspi-gcc-files',
  compiler_files = ':linaro_linux_compiler_files',
  cpu = 'raspi',
  dwp_files = ':empty',
  dynamic_runtime_libs = [':empty'],
  linker_files = ':linaro_linux_linker_files',
  objcopy_files = '//tools/cpp/raspi-toolchain:objcopy',
  static_runtime_libs = [':empty'],
  strip_files = '//tools/cpp/raspi-toolchain:strip',
  supports_param_files = 1,
)
