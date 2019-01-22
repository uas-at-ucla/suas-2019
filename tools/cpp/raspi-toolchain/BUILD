package(default_visibility = ['//tools/cpp:__pkg__'])

filegroup(
  name = 'gcc',
  srcs = [
    '@raspi-toolchain//:gcc',
    'arm-linux-gnueabihf-gcc',
  ],
)

filegroup(
  name = 'ar',
  srcs = [
    '@raspi-toolchain//:ar',
    'arm-linux-gnueabihf-ar',
  ],
)

filegroup(
  name = 'ld',
  srcs = [
    '@raspi-toolchain//:ld',
    'arm-linux-gnueabihf-ld',
  ],
)

filegroup(
  name = 'nm',
  srcs = [
    '@raspi-toolchain//:nm',
    'arm-linux-gnueabihf-nm',
  ],
)

filegroup(
  name = 'objcopy',
  srcs = [
    '@raspi-toolchain//:objcopy',
    'arm-linux-gnueabihf-objcopy',
  ],
)

filegroup(
  name = 'objdump',
  srcs = [
    '@raspi-toolchain//:objdump',
    'arm-linux-gnueabihf-objdump',
  ],
)

filegroup(
  name = 'strip',
  srcs = [
    '@raspi-toolchain//:strip',
    'arm-linux-gnueabihf-strip',
  ],
)

filegroup(
  name = 'as',
  srcs = [
    '@raspi-toolchain//:as',
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
