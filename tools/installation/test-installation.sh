#!/bin/sh

echo "Gazebo takes forever to run if your machine has less than 4 Gb of RAM...";

cd ../../;
#bazel build //src/...;
bazel run @PX4_sitl//:px4_sitl_visualize;
