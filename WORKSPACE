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
  build_file   = 'third_party/libmavconn.BUILD',
  strip_prefix = "mavros-0.31.0",
)

http_archive(
  name         = "lz4",
  urls         = ["https://github.com/lz4/lz4/archive/v1.9.1.zip"],
  sha256       = '5d36605398d1379bde0444513b6db13d4d8a7819fb72436fa270c19ade5a388c',
  build_file = "third_party/lz4.BUILD",
  strip_prefix = "lz4-1.9.1",
)

http_archive(
  name         = "PX4_sitl",
  urls         = ["https://github.com/uas-at-ucla/PX4_sitl/archive/1.0.zip"],
  sha256       = 'b30c3c3b14f592d8ebf2d185530cd8f3b1e00a6e19d3390f7e2a24aad0327ac1',
  strip_prefix = "PX4_sitl-1.0",
)

http_archive(
  name         = "com_github_nelhage_rules_boost",
  urls         = ["https://github.com/uas-at-ucla/rules_boost/archive/1.0.zip"],
  sha256       = 'f3bad4c1a36b92f9f0f61a25323f7f08d8ccf6a6e0bbbd4a75106752d921dd5e',
  strip_prefix = "rules_boost-1.0",
)

http_archive(
  name         = "gtest",
  urls         = ["https://github.com/uas-at-ucla/googletest/archive/1.0.zip"],
  sha256       = '0288c10f15515a1abd471deba02c8b73e69ec98456afa4be6ccdc89c35c50f74',
  strip_prefix = "googletest-1.0",
)

http_archive(
  name         = "rapidjson",
  urls         = ["https://github.com/Tencent/rapidjson/archive/v1.1.0.zip"],
  sha256       = '8e00c38829d6785a2dfb951bb87c6974fa07dfe488aa5b25deec4b8bc0f6a3ab',
  build_file = 'third_party/rapidjson.BUILD',
  strip_prefix = "rapidjson-1.1.0",
)

http_archive(
  name         = "sodium",
  urls         = ["https://github.com/SodiumFRP/sodium-cxx/archive/0.11.0.zip"],
  sha256       = '38eb0583ddcf0f39b2cb53f79930063903f0fe5caef9d37aad5524a7478d4dc4',
  strip_prefix = "sodium-cxx-0.11.0",
)

http_archive(
  name         = "sensor_msgs",
  urls         = ["https://github.com/uas-at-ucla/sensor_msgs/archive/1.0.zip"],
  sha256       = 'd1dbea580b1050d53696f8a3906611070631bd4ee66811c4cda24a9e5d8ce571',
  strip_prefix = "sensor_msgs-1.0",
)

http_archive(
  name         = "mavros_msgs",
  urls         = ["https://github.com/uas-at-ucla/mavros_msgs/archive/1.0.zip"],
  sha256       = 'b2a0e106f4ef6aadbfaa72e926d1370411f36af537a677bb139d4b9add270a73',
  strip_prefix = "mavros_msgs-1.0",
)

http_archive(
  name         = "rpi_ws281x",
  urls         = ["https://github.com/uas-at-ucla/rpi_ws281x/archive/1.0.zip"],
  sha256       = '87e823b261a58874e2c91c22981bea243728380a375c6a05534d64bdcce1a0d0',
  build_file   = 'third_party/rpi_ws281x.BUILD',
  strip_prefix = "rpi_ws281x-1.0",
)

http_archive(
  name         = "socketio_client_cpp",
  urls         = ["https://github.com/uas-at-ucla/socket.io-client-cpp/archive/1.61_1.zip"],
  sha256       = 'ae8827365318dd7f2f25239a880b3d1dcb7befbee8609c71bf08afffcb735d70',
  build_file   = 'third_party/socketio_client_cpp.BUILD',
  strip_prefix = "socket.io-client-cpp-1.61_1",
)

http_archive(
  name         = "matplotlibcpp",
  urls         = ["https://github.com/uas-at-ucla/matplotlib-cpp/archive/1.0.zip"],
  sha256       = '63ab2ed1121e6e00b8410a889a39a34d9dec6ce5b7cc8c6b3d5047f83c20126d',
  build_file   = "third_party/matplotlibcpp.BUILD",
  strip_prefix = "matplotlib-cpp-1.0",
)

http_archive(
  name         = "ros_bazel",
  urls         = ["https://github.com/uas-at-ucla/matplotlib-cpp/archive/1.0.zip"],
  sha256       = '35f57c5a3dfa6f548e28fa13266c10cacad509ce0ad2b6666f137596ccbc985c',
  build_file   = "third_party/ros_bazel.BUILD",
  strip_prefix = "ros_bazel-1.0",
)

http_archive(
  name         = "gphoto2pp",
  urls         = ["https://github.com/uas-at-ucla/gphoto2pp/archive/1.0.zip"],
  sha256       = '318e4df8647b0acc18bb6b46e836b4c8e88e337ab53fa40f101db45db14a3957',
  build_file   = "third_party/gphoto2pp.BUILD",
  strip_prefix = "gphoto2pp-1.0",
)

http_archive(
  name         = "ros_bazel",
  urls         = ["https://github.com/uas-at-ucla/ros_bazel/archive/1.0.zip"],
  sha256       = '35f57c5a3dfa6f548e28fa13266c10cacad509ce0ad2b6666f137596ccbc985c',
  build_file   = "third_party/ros_bazel.BUILD",
  strip_prefix = "ros_bazel-1.0",
)

http_archive(
  name         = "libgphoto2",
  urls         = ["https://github.com/uas-at-ucla/libgphoto2/archive/1.0.zip"],
  sha256       = '8212cc5e8ee75cbfe1b500b8a85ae9515437487afa070eefba5a5e36c6638112',
  build_file   = "third_party/libgphoto2.BUILD",
  strip_prefix = "libgphoto2-1.0",
)

http_archive(
  name         = 'mavlink_v1',
  urls         = ["https://github.com/uas-at-ucla/c_library_v1/archive/1.0.zip"],
  sha256       = '554fe114d91726905ef526cf62dca19824b9d6c148db9a9315116b28dea5c617',
  build_file   = 'third_party/mavlink_v1.BUILD',
  strip_prefix = "c_library_v1-1.0",
)

http_archive(
  name         = 'mavlink_v2',
  urls         = ["https://github.com/uas-at-ucla/c_library_v2/archive/1.0.zip"],
  sha256       = '0d83e01bdb53676c672823dcbb525301af293336befd01b21291a2edca5a5723',
  build_file   = 'third_party/mavlink_v2.BUILD',
  strip_prefix = "c_library_v2-1.0",
)

http_archive(
  name         = 'libmavconn',
  urls         = ["https://github.com/mavlink/mavros/archive/0.31.0.zip"],
  sha256       = '13ab9f8b8a7866801e91a5f7e5702057b82ccfe86ef375835d5bc34ac6631118',
  build_file   = 'third_party/libmavconn.BUILD',
  strip_prefix = "mavros-0.31.0",
)

http_archive(
  name         = 'console_bridge',
  urls         = ["https://github.com/uas-at-ucla/console_bridge/archive/0.4.0_1.zip"],
  sha256       = 'bff636571a63b0415be89bc3f24856a44cc7ffe2b05139e416f9d2063844b4c1',
  build_file   = 'third_party/console_bridge.BUILD',
  strip_prefix = "console_bridge-0.4.0_1",
)

http_archive(
  name         = "libzmq",
  urls         = ["https://github.com/uas-at-ucla/libzmq/archive/4.2.3_1.zip"],
  sha256       = '9f626dc58e345c5cc15cb3d88969fe334d3c881b07eed5d1e94865bde137b290',
  build_file   = "third_party/libzmq.BUILD",
  strip_prefix = "libzmq-4.2.3_1",
)

http_archive(
  name         = "WiringPi",
  urls         = ["https://github.com/uas-at-ucla/WiringPi/archive/1.0.zip"],
  sha256       = '2ace0b5f1a38d1a8ab9a0c48ed451633f32c90f453ccbe02a03c28db2707f3c1',
  build_file   = "third_party/WiringPi.BUILD",
  strip_prefix = "WiringPi-1.0",
)

http_archive(
  name         = "pigpio",
  urls         = ["https://github.com/uas-at-ucla/pigpio/archive/1.0.zip"],
  sha256       = 'ca0d1d03546f4c2f315185fa00382256ae018dd5181583479472fb8376c0b8ef',
  build_file   = "third_party/pigpio.BUILD",
  strip_prefix = "pigpio-1.0",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()
