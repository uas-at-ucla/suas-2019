package(default_visibility = ['//visibility:public'])

filegroup(
  name = 'gcc',
  srcs = [
    'bin/arm-linux-gnueabihf-gcc',
  ],
)

filegroup(
  name = 'ar',
  srcs = [
    'bin/arm-linux-gnueabihf-ar',
  ],
)

filegroup(
  name = 'ld',
  srcs = [
    'bin/arm-linux-gnueabihf-ld',
  ],
)

filegroup(
  name = 'nm',
  srcs = [
    'bin/arm-linux-gnueabihf-nm',
  ],
)

filegroup(
  name = 'objcopy',
  srcs = [
    'bin/arm-linux-gnueabihf-objcopy',
  ],
)

filegroup(
  name = 'objdump',
  srcs = [
    'bin/arm-linux-gnueabihf-objdump',
  ],
)

filegroup(
  name = 'strip',
  srcs = [
    'bin/arm-linux-gnueabihf-strip',
  ],
)

filegroup(
  name = 'as',
  srcs = [
    'bin/arm-linux-gnueabihf-as',
  ],
)

cc_library(
  name = 'librt',
  srcs = [
    'arm-linux-gnueabihf/sysroot/usr/lib/arm-linux-gnueabihf/librt.so',
  ],
)

cc_library(
  name = 'libdl',
  srcs = [
    'arm-linux-gnueabihf/sysroot/usr/lib/arm-linux-gnueabihf/libdl.so',
  ],
)

cc_library(
  name = 'libm',
  srcs = [
    'arm-linux-gnueabihf/sysroot/usr/lib/arm-linux-gnueabihf/libm.so',
  ],
)

cc_library(
  name = 'libpthread',
  deps = [
    '@//tools/cpp/raspi-toolchain:libpthread',
  ],
)

filegroup(
  name = 'compiler_pieces',
  srcs = glob([
    'arm-linux-gnueabihf/**',
    'lib/**',
    '**/*'
  ]),
)

filegroup(
  name = 'compiler_components',
  srcs = [
    ':gcc',
    ':ar',
    ':ld',
    ':nm',
    ':objcopy',
    ':objdump',
    ':strip',
    ':as',
  ],
)
