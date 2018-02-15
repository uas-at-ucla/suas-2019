workspace(name = "com_uclauas")

new_git_repository(
  name = 'gcc_linaro_arm_linux_gnueabihf_raspbian_repo',
  remote = 'https://github.com/uas-at-ucla/raspi-toolchain.git',
  build_file = 'tools/compilers/gcc_linaro_arm_linux_gnueabihf_raspbian.BUILD',
  commit = 'f1c0b5666ab9fdb0cf491a5e18cea312a077f34b',
)

new_git_repository(
  name = 'websocketpp',
  remote = 'https://github.com/zaphoyd/websocketpp.git',
  build_file = 'lib/third_party/websocketpp.BUILD',
  commit = '378437aecdcb1dfe62096ffd5d944bf1f640ccc3',
)

new_git_repository(
  name = 'rapidjson',
  remote = 'https://github.com/Tencent/rapidjson.git',
  build_file = 'lib/third_party/rapidjson.BUILD',
  commit = 'daabb88e001f562e1f7df5f44d7fed32a0c107c2',
)

new_git_repository(
  name = 'rpi_ws281x',
  remote = 'https://github.com/uas-at-ucla/rpi_ws281x.git',
  build_file = 'lib/third_party/rpi_ws281x.BUILD',
  commit = 'e3345674e5405777b61f4f509cd817d4923e99e4',
)

new_git_repository(
  name = 'socketio_client_cpp',
  remote = 'https://github.com/socketio/socket.io-client-cpp.git',
  build_file = 'lib/third_party/socketio_client_cpp.BUILD',
  commit = '6063cb1d612f6ca0232d4134a018053fb8faea20',
)

new_git_repository(
  name = 'mavlink_v1',
  remote = 'https://github.com/mavlink/c_library_v1.git',
  build_file = 'lib/third_party/mavlink_v1.BUILD',
  commit = 'c0e0ebf80ce0e0d497245f94869eb9c5e817114f',
)

new_git_repository(
  name = 'libmavconn',
  remote = 'https://github.com/mavlink/mavros.git',
  build_file = 'lib/third_party/libmavconn.BUILD',
  commit = '306f7ee6e0d90fa2c63f4a821b8f09b0f02219b9',
)

new_git_repository(
  name = 'eigen',
  remote = 'https://github.com/PX4/eigen.git',
  build_file = 'lib/third_party/eigen.BUILD',
  commit = 'e7850ed81f9c469e02df496ef09ae32ec0379b71',
)

new_git_repository(
  name = 'console_bridge',
  remote = 'https://github.com/uas-at-ucla/console_bridge.git',
  build_file = 'lib/third_party/console_bridge.BUILD',
  commit = '57f9d41e3402d317fec85b3cc531d9dc4437bf45',
)

new_http_archive(
  name = "gtest",
  url = "https://github.com/google/googletest/archive/release-1.8.0.zip",
  sha256 = "f3ed3b58511efd272eb074a3a6d6fb79d7c2e6a0e374323d1e6bcbcc1ef141bf",
  build_file = "lib/third_party/gtest.BUILD",
  strip_prefix = "googletest-release-1.8.0",
)

git_repository(
  name = "PX4_sitl",
  remote = "https://github.com/uas-at-ucla/PX4_sitl.git",
  commit = "e9aa8c07cd9b62e484512826ea4bdab396be3801",
)

git_repository(
  name = "com_github_nelhage_rules_boost",
  commit = "239ce40e42ab0e3fe7ce84c2e9303ff8a277c41a",
  remote = "https://github.com/nelhage/rules_boost",
)

git_repository(
  name = "protobuf",
  commit = "07f023188e929019f506e9b390dde70539ea857f",
  remote = "https://github.com/google/protobuf.git",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()
