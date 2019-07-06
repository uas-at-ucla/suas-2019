workspace(name = "com_uclauas")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

new_local_repository(
    name = "python_linux",
    path = "/usr",
    build_file_content = """
cc_library(
    name = "python27-lib",
    srcs = ["lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so"],
    hdrs = glob(["include/python2.7/*.h"]),
    includes = ["include/python2.7"],
    visibility = ["//visibility:public"]
)
    """
)

local_repository(
  name = "ignore_server_rxjs",
  path = "./src/ground/server/node_modules/rxjs/src",
)

local_repository(
  name = "ignore_ui_rxjs",
  path = "./src/ground/ui/node_modules/rxjs/src",
)

new_git_repository(
  name = 'rapidjson',
  remote = 'https://github.com/Tencent/rapidjson.git',
  build_file = 'third_party/rapidjson.BUILD',
  commit = 'daabb88e001f562e1f7df5f44d7fed32a0c107c2',
)

new_git_repository(
  name = 'rpi_ws281x',
  remote = 'https://github.com/uas-at-ucla/rpi_ws281x.git',
  build_file = 'third_party/rpi_ws281x.BUILD',
  commit = '74927784671de0c839e41b4525876455777393da',
)

new_git_repository(
  name = 'socketio_client_cpp',
  remote = 'https://github.com/uas-at-ucla/socket.io-client-cpp.git',
  build_file = 'third_party/socketio_client_cpp.BUILD',
  commit = '28d2105f8052641498d305cb5ea02a73e60eb91a',
)

new_git_repository(
  name = 'mavlink_v1',
  remote = 'https://github.com/mavlink/c_library_v1.git',
  build_file = 'third_party/mavlink_v1.BUILD',
  commit = 'c0e0ebf80ce0e0d497245f94869eb9c5e817114f',
)

new_git_repository(
  name = 'mavlink_v2',
  remote = 'https://github.com/mavlink/c_library_v2.git',
  build_file = 'third_party/mavlink_v2.BUILD',
  commit = '748192f661d0df3763501cfc432861d981952921',
)

new_git_repository(
  name = 'libmavconn',
  remote = 'https://github.com/mavlink/mavros.git',
  build_file = 'third_party/libmavconn.BUILD',
  commit = '306f7ee6e0d90fa2c63f4a821b8f09b0f02219b9',
)

new_git_repository(
  name = 'console_bridge',
  remote = 'https://github.com/uas-at-ucla/console_bridge.git',
  build_file = 'third_party/console_bridge.BUILD',
  commit = '57f9d41e3402d317fec85b3cc531d9dc4437bf45',
)

new_git_repository(
  name = "libzmq",
  commit = "b1fd3b2740f4b18ee25aefe193b31e319fcfbbe5",
  build_file = "third_party/libzmq.BUILD",
  remote = "https://github.com/uas-at-ucla/libzmq.git",
)

new_git_repository(
  name = "sodium",
  commit = "e25e2884e2543f608de34fb92d719f0724d5a5c7",
  build_file = "third_party/sodium.BUILD",
  remote = "https://github.com/SodiumFRP/sodium-cxx.git",
)

new_git_repository(
  name = "matplotlibcpp",
  commit = "9523030a3dc11c05b6cc38743299e4453ec36458",
  build_file = "third_party/matplotlibcpp.BUILD",
  remote = "https://github.com/uas-at-ucla/matplotlib-cpp.git",
)

new_git_repository(
  name = "WiringPi",
  commit = "0f7d03d9f04c226192c299dafe03e1d56ec219c9",
  remote = "https://github.com/WiringPi/WiringPi.git",
  build_file = "third_party/WiringPi.BUILD",
)

new_git_repository(
  name = "pigpio",
  commit = "2125680045f54bed198636222999d053b738b1bf",
  remote = "https://github.com/uas-at-ucla/pigpio.git",
  build_file = "third_party/pigpio.BUILD",
)

new_git_repository(
  name = "libgphoto2",
  commit = "a9f979459f0a64ff356318cd48b6f5b817850ec5",
  remote = "https://github.com/uas-at-ucla/libgphoto2.git",
  build_file = "third_party/libgphoto2.BUILD",
)

new_git_repository(
  name = "gphoto2pp",
  commit = "da6db6f79edc4291f7a5ff38d825b8f3102d6270",
  remote = "https://github.com/maldworth/gphoto2pp.git",
  build_file = "third_party/gphoto2pp.BUILD"
)

new_git_repository(
  name = "ros_bazel",
  commit = "9435753c27f9f3c5fa5790e5fa3da382259fda2f",
  remote = "https://github.com/uas-at-ucla/ros_bazel.git",
  build_file = "third_party/ros_bazel.BUILD"
)

git_repository(
  name = 'mavros_msgs',
  remote = 'https://github.com/uas-at-ucla/mavros_msgs.git',
  commit = '06ed539e9422a8ddf877daa066842303a1b9591f',
)

git_repository(
  name = 'sensor_msgs',
  remote = 'https://github.com/uas-at-ucla/sensor_msgs.git',
  commit = '7926ae1b63c33e9207483253f2911e5551418afe',
)

git_repository(
  name = "gtest",
  commit = "d85b96280a55aa70ace9b86c9773cfe6634a1686",
  remote = "https://github.com/uas-at-ucla/googletest.git"
)

git_repository(
  name = "com_github_nelhage_rules_boost",
  commit = "0a63e8f7939024bb4acef9c9daaa27956bc94c61",
  remote = "https://github.com/uas-at-ucla/rules_boost.git",
)

git_repository(
  name = "PX4_sitl",
  remote = "https://github.com/uas-at-ucla/PX4_sitl.git",
  commit = "e9aa8c07cd9b62e484512826ea4bdab396be3801",
)

http_archive(
  name         = 'raspi_toolchain',
  build_file   = 'compilers/raspi_toolchain.BUILD',
  sha256       = '2dfeb5e8f9d7b2e9c67152440435c909e9196a86a7d294a5a15088cba732ba67',
  urls         = ['https://github.com/uas-at-ucla/raspi-toolchain/releases/download/1.1/raspi_toolchain.zip'],
  strip_prefix = 'raspi_toolchain',
)

http_archive(
  name         = "eigen",
  build_file   = 'third_party/eigen.BUILD',
  sha256       = '3f1726bde240923e786b003fa5f87874cfdc61251e964002e0e03781186e92f1',
  urls         = ['https://github.com/PX4/eigen/archive/3.3.4.zip'],
  strip_prefix = 'eigen-3.3.4',
)

http_archive(
  name         = "cppzmq",
  build_file   = 'third_party/cppzmq.BUILD',
  sha256       = 'ee77e9e7bef2d3942edad4717d5ddd53e95b899757bc06c636e49ba42ef83eae',
  urls         = ['https://github.com/zeromq/cppzmq/archive/v4.3.0.zip'],
  strip_prefix = 'cppzmq-4.3.0',
)

http_archive(
  name         = "flann",
  build_file   = 'third_party/flann.BUILD',
  sha256       = '45420be368a76d1ea1d4337e8bd9b031d75a1abc01a1120a7ab9ea72a922e45f',
  urls         = ['https://github.com/mariusmuja/flann/archive/1.9.1.zip'],
  strip_prefix = 'flann-1.9.1',
)

http_archive(
  name         = 'spdlog',
  build_file   = 'third_party/spdlog.BUILD',
  urls         = ['https://github.com/gabime/spdlog/archive/v1.3.1.zip'],
  strip_prefix = 'spdlog-1.3.1',
)

http_archive(
  name         = "com_google_protobuf",
  build_file   = 'third_party/com_google_protobuf.BUILD',
  sha256       = '9510dd2afc29e7245e9e884336f848c8a6600a14ae726adb6befdb4f786f0be2',
  urls         = ['https://github.com/google/protobuf/archive/v3.6.1.3.zip'],
  strip_prefix = 'protobuf-3.6.1.3',
)

http_archive(
  name         = 'genmsg_repo',
  build_file   = 'third_party/genmsg.BUILD',
  sha256       ='d7627a2df169e4e8208347d9215e47c723a015b67ef3ed8cda8b61b6cfbf94d2',
  urls         = ['https://github.com/ros/genmsg/archive/0.5.8.tar.gz'],
  strip_prefix = 'genmsg-0.5.8',
)

http_archive(
  name         = 'genpy_repo',
  build_file   = 'third_party/genpy.BUILD',
  sha256       = '35e5cd2032f52a1f77190df5c31c02134dc460bfeda3f28b5a860a95309342b9',
  urls         = ['https://github.com/ros/genpy/archive/0.6.5.tar.gz'],
  strip_prefix = 'genpy-0.6.5',
)

http_archive(
  name         = 'gencpp',
  build_file   = 'third_party/gencpp.BUILD',
  sha256       = 'd7627a2df169e4e8208347d9215e47c723a015b67ef3ed8cda8b61b6cfbf94d2',
  urls         = ['https://github.com/ros/gencpp/archive/0.5.5.tar.gz'],
  strip_prefix ='gencpp-0.5.5',
)

http_archive(
  name         = 'GeographicLib',
  build_file   = 'third_party/GeographicLib.BUILD',
  sha256       = '7cafd8d9e3a95de81bff5b967157af474a64cc2507757facf8362a58d6aaf3b3',
  urls         = ['https://github.com/Sciumo/GeographicLib/archive/v1.43.zip'],
  strip_prefix = 'GeographicLib-1.43',
)

http_archive(
  name         = "com_google_absl",
  urls         = ["https://github.com/abseil/abseil-cpp/archive/master.zip"],
  sha256       = '8b47c06f6e30920fc90018827bfa54bc649cc61b35a25a261629c9edc2a14611',
  strip_prefix = "abseil-cpp-master",
)

http_archive(
  name         = "websocketpp",
  build_file   = 'third_party/websocketpp.BUILD',
  sha256       = '80f49469f1702ec64ef5d8e71302f6b781dca8ae06e560421cab42961c8c7ce6',
  urls         = ["https://github.com/zaphoyd/websocketpp/archive/0.8.1.zip"],
  strip_prefix = "websocketpp-0.8.1",
)

http_archive(
  name         = "libmavconn",
  urls         = ["https://github.com/mavlink/mavros/archive/0.31.0.zip"],
  build_file = 'third_party/libmavconn.BUILD',
  strip_prefix = "mavros-0.31.0",
)

http_archive(
  name         = "lz4",
  urls         = ["https://github.com/lz4/lz4/archive/v1.9.1.zip"],
  sha256       = '5d36605398d1379bde0444513b6db13d4d8a7819fb72436fa270c19ade5a388c',
  build_file = "third_party/lz4.BUILD",
  strip_prefix = "lz4-1.9.1",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()
