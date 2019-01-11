#!/bin/bash

echo "Waiting for PX4 simulator docker to start..."

DOCKER_IP=`docker network inspect uas_bridge | grep -o -P ".{0,9}2/16" | sed 's/\/16//g'`

unset PX4_RUNNING_CONTAINER
while [ -z $PX4_RUNNING_CONTAINER ]
do
  PX4_RUNNING_CONTAINER=$(docker ps \
    --filter status=running \
    --filter name="uas-at-ucla_px4-simulator" \
    --format "{{.ID}}" \
    --latest
  )

  sleep 0.5
done

echo "PX4 simulator docker started!"

docker exec -t $PX4_RUNNING_CONTAINER \
  su - uas bash -c "
  HOST_IP=\$(/sbin/ip route|awk '/default/ { print \$3 }')
  mavlink-routerd -e \$HOST_IP:8086 -e $DOCKER_IP:8084 0.0.0.0:14550"
