#!/bin/bash

CONTAINER_NAME="uas-at-ucla_ros-rqt"

kill_rqt_if_running() {
  ENV_DOCKER_RUNNING_CONTAINER="$(docker ps \
    --filter name=$CONTAINER_NAME \
    --filter status=running \
    --format "{{.ID}}" \
    --latest)"

  if [ ! -z "$ENV_DOCKER_RUNNING_CONTAINER" ]
  then
    docker kill $CONTAINER_NAME
  fi
}

trap kill_rqt_if_running INT

kill_rqt_if_running

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

UAS_AT_UCLA_CONTROLS_CONTAINER_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' uas-at-ucla_controls 2> /dev/null)

ENV_DOCKER_CONTAINER="$(docker ps \
  --filter name=$CONTAINER_NAME \
  --format "{{.ID}}" \
  --latest)"

if [ ! -z "$ENV_DOCKER_CONTAINER" ]
then
  docker rm $CONTAINER_NAME
fi

docker run \
  -t \
  --rm \
  -v $XSOCK:${XSOCK}:rw \
  -v $XAUTH:${XAUTH}:rw \
  -e XAUTHORITY=${XAUTH} \
  -e DISPLAY \
  -e ROS_MASTER_URI="http://$UAS_AT_UCLA_CONTROLS_CONTAINER_IP:11311" \
  --name $CONTAINER_NAME \
  --net uas_bridge  \
  diegoferigo/ros \
  rqt

kill_rqt_if_running
