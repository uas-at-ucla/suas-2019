#!/bin/sh

cd ../../;
bazel test //src/...;
#bazel run @PX4_sitl//:px4_sitl_visualize;
