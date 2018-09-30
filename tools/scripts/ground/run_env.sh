#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

unset ENV_DOCKER_RUNNING_CONTAINER
unset ENV_DOCKER_CONTAINER
unset RUNNING_DOCKER_CONTAINERS

ENV_DOCKER_RUNNING_CONTAINER=$(docker ps \
  --filter name=uas-at-ucla_ground \
  --filter status=running \
  --format "{{.ID}}" \
  --latest \
  )

if [ ! -z $ENV_DOCKER_RUNNING_CONTAINER ]
then
  echo "Docker ground environment already running."

  exit
fi

ENV_DOCKER_CONTAINER=$(docker ps \
  --filter name=uas-at-ucla_ground \
  --format "{{.ID}}" \
  --latest
  )

if [ ! -z $ENV_DOCKER_CONTAINER ]
then
  echo "Removing old container with ID $ENV_DOCKER_CONTAINER"
  docker rm $ENV_DOCKER_CONTAINER
fi


# Build docker container.
if [[ -z $TRAVIS ]]
then
  docker build -t uas-at-ucla_ground tools/dockerfiles/ground
else
  docker build -t uas-at-ucla_ground tools/dockerfiles/ground > /dev/null
fi

if [ $? -ne 0 ]
then
    echo "Error building UAS ground docker container."
    exit 1
fi

# Create network for docker container to use.
docker network create -d bridge uas_bridge > /dev/null 2>&1 || true

# Start docker container and let it run forever.
PLATFORM=$(uname -s)
DOCKER_RUN_CMD="set -x; \
  getent group $(id -g) || groupadd -g $(id -g) host_group; \
  mkdir -p /tmp/home/uas; \
  usermod -d /tmp/home/uas uas; \
  usermod -u $(id -u) -g $(id -g) uas; \
  usermod -d /home/uas uas
  echo STARTED > /tmp/uas_init; \
  sudo -u uas bash -c \"
  sleep infinity\""

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
  -d \
  --rm \
  --net uas_bridge \
  -p 3000:3000 \
  -v $ROOT_PATH:/home/uas/code_env \
  -v $ROOT_PATH/tools/cache/bazel:/home/uas/.cache/bazel  \
  --dns 8.8.8.8 \
  --name uas-at-ucla_ground \
  uas-at-ucla_ground \
  bash -c "$DOCKER_RUN_CMD"

echo "Started uas ground docker image. Waiting for it to boot..."

# Wait for docker container to start up.
while [ -z $RUNNING_DOCKER_CONTAINERS ]
do
  RUNNING_DOCKER_CONTAINERS=$(docker ps \
    --filter name=uas-at-ucla_ground \
    --filter status=running \
    --format "{{.ID}}" \
    --latest \
  )

  sleep 0.25
done

# Wait for permission scripts to execute.
./tools/scripts/ground/exec.sh "while [ ! -f /tmp/uas_init ];do sleep 0.25;done"
