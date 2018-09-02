#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

unset UAS_AT_UCLA_SIM_DOCKER_RUNNING_CONTAINER
unset UAS_AT_UCLA_SIM_DOCKER_CONTAINER

UAS_AT_UCLA_SIM_DOCKER_RUNNING_CONTAINER=$(docker ps \
  --filter name=uas_sim \
  --filter status=running \
  --format "{{.ID}}" \
  --latest \
  )

if [ ! -z $UAS_AT_UCLA_SIM_DOCKER_RUNNING_CONTAINER ]
then
  # Kill running docker simulators.
  docker kill $UAS_AT_UCLA_SIM_DOCKER_RUNNING_CONTAINER
fi

UAS_AT_UCLA_SIM_DOCKER_CONTAINER=$(docker ps \
  --filter name=uas_sim \
  --format "{{.ID}}" \
  --latest
  )

if [ ! -z $UAS_AT_UCLA_SIM_DOCKER_CONTAINER ]
then
  echo "Removing old container with ID $UAS_AT_UCLA_SIM_DOCKER_CONTAINER"
  docker rm $UAS_AT_UCLA_SIM_DOCKER_CONTAINER
fi

# Create network for docker container to use.
docker network create -d bridge uas_bridge > /dev/null || true

# Enable access to xhost from the container
xhost +

# Start docker container and let it run forever.
docker run \
  --rm -t \
  --env=LOCAL_USER_ID="$(id -u)" \
  -v $(pwd)/tools/cache/px4_firmware:/src/firmware/:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e DISPLAY=:0 \
  --name uas_sim \
  --net uas_bridge \
  -p 14556:14556/udp \
  px4io/px4-dev-ros:2017-10-23 \
  /bin/sh -c "cd /src/firmware;HEADLESS=1 make posix_sitl_default gazebo"
