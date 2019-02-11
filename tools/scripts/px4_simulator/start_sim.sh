#!/bin/bash

set -e

source tools/scripts/docker/start_machine_mac.sh

docker build -t uas-at-ucla_px4-simulator tools/dockerfiles/px4_simulator

if [ ! -d tools/cache/px4_simulator ]
then
  git clone \
    https://github.com/uas-at-ucla/Firmware.git \
    tools/cache/px4_simulator
fi

docker run \
  -it \
  --rm \
  --name uas-at-ucla_px4-simulator \
  -v $(pwd)/tools/cache/px4_simulator:/home/uas/px4_simulator \
  -p 8085:8085/udp \
  -p 8095:8095/udp \
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
  make posix_sitl_default jmavsim\""
