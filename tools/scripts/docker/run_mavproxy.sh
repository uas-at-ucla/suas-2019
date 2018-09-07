#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

SIM_IP=""

while true
do
  SIM_IP="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' uas_sim 2> /dev/null)"
  if [ $? == 0 ]
  then
    SIM_IP="udpout:$SIM_IP:14557"
    echo $SIM_IP

    ./tools/scripts/docker/exec.sh \
      rm -rf /tmp/mav* && \
      ./tools/scripts/docker/exec.sh \
      /home/uas/.local/bin/mavproxy.py \
      --state-basedir=/tmp \
      --nowait \
      --show-errors \
      --master $SIM_IP \
      --out udp:0.0.0.0:8084 \
      --out tcp:172.19.0.2:8090 \
      --baud 921600

    exit 0
  else
    echo "Waiting for Pixhawk sim to start..."
    sleep 1
  fi
done
