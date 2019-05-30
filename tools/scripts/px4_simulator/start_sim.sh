#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

set -e

docker build -t uas-at-ucla_px4-simulator tools/dockerfiles/px4_simulator

if [ ! -d tools/cache/px4_simulator ]
then
  git clone \
    https://github.com/uas-at-ucla/Firmware.git \
    tools/cache/px4_simulator
fi

# Set root path of the repository volume on the host machine.
# Note: If docker is called within another docker instance & is trying to start
#       the UAS@UCLA docker environment, the root will need to be set to the
#       path that is used by wherever dockerd is running.
ROOT_PATH=$(pwd)
if [ ! -z $HOST_ROOT_SEARCH ] && [ ! -z $HOST_ROOT_REPLACE ]
then
  # Need to use path of the host container running dockerd.
  ROOT_PATH=${ROOT_PATH/$HOST_ROOT_SEARCH/$HOST_ROOT_REPLACE}
fi

echo "Root path is $ROOT_PATH"

docker run \
  -it \
  --rm \
  --name uas-at-ucla_px4-simulator \
  -v $ROOT_PATH/tools/cache/px4_simulator:/home/uas/px4_simulator \
  -p 8085:8085/udp \
  -p 8095:8095/udp \
  -p 5760:5760 \
  --net uas_bridge \
  uas-at-ucla_px4-simulator \
  bash -c "
  set -x
  getent group $(id -g) || groupadd -g $(id -g) host_group
  usermod -u $(id -u) -g $(id -g) uas
  Xvfb :1 -screen 0 1024x768x16 &
  DISPLAY=:1.0
  export DISPLAY
  su - uas bash -c \"
  cd px4_simulator
  HOST_IP=\\\$(/sbin/ip route|awk '/default/ { print \\\$3 }')
  export PX4_HOME_LAT=38.147483
  export PX4_HOME_LON=-76.427778
  export PX4_HOME_ALT=40
  make px4_sitl_default jmavsim\""
