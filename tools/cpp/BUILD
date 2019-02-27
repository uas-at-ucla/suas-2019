package(default_visibility = ['//visibility:public'])

filegroup(
  name = "toolchain_fg",
  srcs = [
    ":cc-compiler-k8",
    ":cc-compiler-raspi",
    ":raspi-all-files",
    '@raspi_toolchain//:compiler_components',
  ],
)

cc_toolchain_suite(
  name = 'toolchain',
  # target_cpu | compiler
  toolchains = {
    'raspi': ':cc-compiler-raspi',
    'k8': ':cc-compiler-k8',
    'raspi|gcc': ':cc-compiler-raspi',
    'k8|compiler': ':cc-compiler-k8',
  },
)

filegroup(
    name = "empty",
    srcs = [],
)

cc_toolchain(
  name = "cc-compiler-k8",
  toolchain_identifier = "local",
  all_files = ":empty",
  ar_files = ":empty",
  as_files = ":empty",
  compiler_files = ":empty",
  cpu = "local",
  dwp_files = ":empty",
  linker_files = ":empty",
  objcopy_files = ":empty",
  strip_files = ":empty",
  supports_param_files = 1,
)

cc_toolchain(
  name = 'cc-compiler-raspi',
  toolchain_identifier = "raspi",  
  all_files = ':raspi-all-files',
  compiler_files = ':raspi_compiler_files',
  cpu = 'raspi',
  dwp_files = ':empty',
  linker_files = ':raspi_linker_files',
  objcopy_files = '//tools/cpp/raspi_toolchain:objcopy',
  strip_files = '//tools/cpp/raspi_toolchain:strip',
  supports_param_files = 1,
)

filegroup(
  name = 'raspi-all-files',
  srcs = [
    '//tools/cpp/raspi_toolchain:tool-wrappers',
    '@raspi_toolchain//:compiler_pieces',
  ],
)

filegroup(
  name = 'raspi_linker_files',
  srcs = [
    '//tools/cpp/raspi_toolchain:ar',
    '//tools/cpp/raspi_toolchain:gcc',
    '//tools/cpp/raspi_toolchain:ld',
    '@raspi_toolchain//:compiler_pieces',
  ],
)

filegroup(
  name = 'raspi_compiler_files',
  srcs = [
    '//tools/cpp/raspi_toolchain:as',
    '//tools/cpp/raspi_toolchain:gcc',
    '//tools/cpp/raspi_toolchain:ld',
  ],
)
