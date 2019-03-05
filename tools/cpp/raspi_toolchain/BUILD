package(default_visibility = ['//tools/cpp:__pkg__'])

filegroup(
  name = 'gcc',
  srcs = [
    '@raspi_toolchain//:gcc',
    'arm-linux-gnueabihf-gcc',
  ],
)

filegroup(
  name = 'ar',
  srcs = [
    '@raspi_toolchain//:ar',
    'arm-linux-gnueabihf-ar',
  ],
)

filegroup(
  name = 'ld',
  srcs = [
    '@raspi_toolchain//:ld',
    'arm-linux-gnueabihf-ld',
  ],
)

filegroup(
  name = 'nm',
  srcs = [
    '@raspi_toolchain//:nm',
    'arm-linux-gnueabihf-nm',
  ],
)

filegroup(
  name = 'objcopy',
  srcs = [
    '@raspi_toolchain//:objcopy',
    'arm-linux-gnueabihf-objcopy',
  ],
)

filegroup(
  name = 'objdump',
  srcs = [
    '@raspi_toolchain//:objdump',
    'arm-linux-gnueabihf-objdump',
  ],
)

filegroup(
  name = 'strip',
  srcs = [
    '@raspi_toolchain//:strip',
    'arm-linux-gnueabihf-strip',
  ],
)

filegroup(
  name = 'as',
  srcs = [
    '@raspi_toolchain//:as',
    'arm-linux-gnueabihf-as',
  ],
)

filegroup(
  name = 'tool-wrappers',
  srcs = [
    ':ar',
    ':as',
    ':gcc',
    ':ld',
    ':nm',
    ':objcopy',
    ':objdump',
    ':strip',
  ],
)
