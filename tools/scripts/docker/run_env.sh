#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

unset UAS_AT_UCLA_ENV_DOCKER_RUNNING_CONTAINER
unset UAS_AT_UCLA_ENV_DOCKER_CONTAINER
unset UAS_AT_UCLA_RUNNING_DOCKER_CONTAINERS

UAS_AT_UCLA_ENV_DOCKER_RUNNING_CONTAINER=$(docker ps \
  --filter name=uas_env \
  --filter status=running \
  --format "{{.ID}}" \
  --latest \
  )

if [ ! -z $UAS_AT_UCLA_ENV_DOCKER_RUNNING_CONTAINER ]
then
  echo "Docker environment already running."

  exit
fi

UAS_AT_UCLA_ENV_DOCKER_CONTAINER=$(docker ps \
  --filter name=uas_env \
  --format "{{.ID}}" \
  --latest
  )

if [ ! -z $UAS_AT_UCLA_ENV_DOCKER_CONTAINER ]
then
  echo "Removing old container with ID $UAS_AT_UCLA_ENV_DOCKER_CONTAINER"
  docker rm $UAS_AT_UCLA_ENV_DOCKER_CONTAINER
fi


# Build docker container.
if [ -z $TRAVIS ]
then
  docker build -t uas-at-ucla_software tools/docker
else
  docker build -t uas-at-ucla_software tools/docker > /dev/null
fi

if [ $? -ne 0 ]; then
    echo "Error building UAS env docker container."
    exit 1
fi

# Create network for docker container to use.
docker network create -d bridge uas_bridge > /dev/null 2>&1 || true

mkdir -p tools/docker/cache/bazel

# Start docker container and let it run forever.
PLATFORM=$(uname -s)
DOCKER_BUILD_CMD="set -x; getent group $(id -g) || groupadd -g $(id -g) host_group;usermod -u $(id -u) -g $(id -g) uas;chown -R uas /home/uas/.cache/bazel;echo STARTED > /tmp/uas_init;sudo -u uas bash -c \"bazel;sleep infinity\""

if [ -z $JENKINS_HOST_ROOT ]
then
  ROOT_PATH=$(pwd)
  ROOT_PATH=$JENKINS_HOST_ROOT/${ROOT_PATH:20}
else
  ROOT_PATH=$(pwd)
fi

docker run \
  -d \
  --rm \
  --net uas_bridge \
  -v $ROOT_PATH:/home/uas/code_env/ \
  -v $ROOT_PATH/tools/docker/cache/bazel:/home/uas/.cache/bazel  \
  --dns 8.8.8.8 \
  --name uas_env \
  uas-at-ucla_software \
  bash -c "$DOCKER_BUILD_CMD"

echo "Started uas env docker image. Waiting for it to boot..."

# Wait for docker container to start up.
while [ -z $UAS_AT_UCLA_RUNNING_DOCKER_CONTAINERS ]
do
  UAS_AT_UCLA_RUNNING_DOCKER_CONTAINERS=$(docker ps \
    --filter name=uas_env \
    --filter status=running \
    --format "{{.ID}}" \
    --latest \
  )

  sleep 0.25
done

# Wait for permission scripts to execute.
./tools/scripts/docker/exec.sh "while [ ! -f /tmp/uas_init ];do sleep 0.25;done"
