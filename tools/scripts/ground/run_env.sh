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
./tools/scripts/docker/create_network.sh > /dev/null 2>&1 || true

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

# Start docker container and let it run forever.
PLATFORM=$(uname -s)
DOCKER_RUN_CMD="set -x; \
  echo STARTED > /tmp/uas_init; \
  mkdir -p /tmp/home/uas; \
  usermod -d /tmp/home/uas uas; \
  usermod -u $(id -u) -g $(id -g) uas; \
  usermod -d /home/uas uas; \
  chown uas /home/uas; \
  sleep infinity"

docker run \
  --rm \
  -d \
  --ip 192.168.1.10 \
  -p 3000:3000 \
  -p 8081:8081 \
  -v $ROOT_PATH:/home/uas/code_env \
  --name uas-at-ucla_ground \
  --net uas_bridge \
  --dns 8.8.8.8 \
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

# Wait for setup scripts to execute.
./tools/scripts/ground/exec.sh "while [ ! -f /tmp/uas_init ];do sleep 0.25;done"
