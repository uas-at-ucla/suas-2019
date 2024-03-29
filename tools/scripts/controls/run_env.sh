#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

unset ENV_DOCKER_RUNNING_CONTAINER
unset ENV_DOCKER_CONTAINER
unset RUNNING_DOCKER_CONTAINERS

# Check if Dockerfile was updated, indicating that the existing docker container
# needs to be killed.
mkdir -p tools/cache/checksums
CHECKSUM=$(md5sum tools/dockerfiles/controls/Dockerfile)
MATCH=0

if [ -f tools/cache/checksums/controls_dockerfile.md5 ]
then
  LAST_CHECKSUM=$(cat tools/cache/checksums/controls_dockerfile.md5)

  CHECKSUM=$(echo -e "${CHECKSUM}" | tr -d '[:space:]')
  LAST_CHECKSUM=$(echo -e "${LAST_CHECKSUM}" | tr -d '[:space:]')

  if [ "$CHECKSUM" = "$LAST_CHECKSUM" ]
  then
    MATCH=1
  fi
fi

if [ $MATCH -eq 0 ]
then
  RUNNING_DOCKER_CONTAINERS=$(docker ps \
    --filter name=uas-at-ucla_controls \
    --filter status=running \
    --format "{{.ID}}" \
    --latest \
  )

  if [ ! -z $RUNNING_DOCKER_CONTAINERS ]
  then
    echo "controls dockerfile updated; killing existing container"

    docker kill $RUNNING_DOCKER_CONTAINERS

    while [ ! -z $RUNNING_DOCKER_CONTAINERS ]
    do
      RUNNING_DOCKER_CONTAINERS=$(docker ps \
        --filter name=uas-at-ucla_controls \
        --filter status=running \
        --format "{{.ID}}" \
        --latest \
      )

      sleep 0.25
    done
  fi

  CHECKSUM=$(md5sum tools/dockerfiles/controls/Dockerfile)
  echo $CHECKSUM > tools/cache/checksums/controls_dockerfile.md5
fi

ENV_DOCKER_RUNNING_CONTAINER=$(docker ps \
  --filter name=uas-at-ucla_controls \
  --filter status=running \
  --format "{{.ID}}" \
  --latest \
  )

if [ ! -z $ENV_DOCKER_RUNNING_CONTAINER ]
then
  echo "Docker environment already running."

  exit
fi

ENV_DOCKER_CONTAINER=$(docker ps \
  --filter name=uas-at-ucla_controls \
  --format "{{.ID}}" \
  --latest
  )

if [ ! -z $ENV_DOCKER_CONTAINER ]
then
  echo "Removing old container with ID $ENV_DOCKER_CONTAINER"
  docker rm $ENV_DOCKER_CONTAINER
fi

BUILD_FLAGS="-t uas-at-ucla_controls"

while test $# -gt 0
do
    case "$1" in
        --rebuild) BUILD_FLAGS="$BUILD_FLAGS --no-cache"
            ;;
    esac
    shift
done

# Build docker container.
docker build $BUILD_FLAGS tools/dockerfiles/controls
# if [[ -z $TRAVIS ]]
# then
#   docker build $BUILD_FLAGS tools/dockerfiles/controls
# else
#   docker build $BUILD_FLAGS tools/dockerfiles/controls > /dev/null
# fi

if [ $? -ne 0 ]
then
    echo "Error building UAS env docker container."
    exit 1
fi

# Create network for docker container to use.
./tools/scripts/docker/create_network.sh > /dev/null 2>&1 || true

mkdir -p tools/cache/bazel
pwd

# Set root path of the repository volume on the host machine.
# Note: If docker is called within another docker instance & is trying to start
#       the UAS@UCLA docker environment, the root will need to be set to the
#       path that is used by wherever dockerd is running.
ROOT_PATH=$(pwd)
GLOBAL_FOLDER=$(pwd)
if [ ! -z $HOST_ROOT_SEARCH ] && [ ! -z $HOST_ROOT_REPLACE ]
then
  # Need to use path of the host container running dockerd.
  ROOT_PATH=${ROOT_PATH/$HOST_ROOT_SEARCH/$HOST_ROOT_REPLACE}

  # Set up global folder.
  mkdir -p $HOST_ROOT_SEARCH/global
  GLOBAL_FOLDER=$HOST_ROOT_REPLACE/global
fi

echo "Root path is $ROOT_PATH"

# Start docker container and let it run forever.
PLATFORM=$(uname -s)
DOCKER_BUILD_CMD="set -x; \
  getent group $(id -g) || groupadd -g $(id -g) host_group; \
  mkdir -p /tmp/home/uas; \
  usermod -d /tmp/home/uas uas; \
  usermod -u $(id -u) -g $(id -g) uas; \
  usermod -d /home/uas uas; \
  chown uas /home/uas; \
  chown uas /home/uas/.cache; \
  echo STARTED > /tmp/uas_init; \
  sudo -u uas bash -c \"bazel; \
  source /home/uas/.bashrc; \
  /opt/ros/melodic/bin/roscore &> /dev/null; \
  sleep infinity\""

docker run                                          \
  -d                                                \
  --rm                                              \
  --cap-add=SYS_PTRACE                              \
  --security-opt seccomp=unconfined                 \
  --net uas_bridge                                  \
  --ip 192.168.3.20                                 \
  -v $ROOT_PATH:/home/uas/code_env                  \
  -v ~/.ssh:/home/uas/.ssh                          \
  -v /tmp/.X11-unix:/tmp/.X11-unix                  \
  -v $GLOBAL_FOLDER:/tmp/uasatucla                  \
  -e DISPLAY=$DISPLAY                               \
  -e CONTINUOUS_INTEGRATION=$CONTINUOUS_INTEGRATION \
  -e JENKINS_URL=$JENKINS_URL                       \
  --privileged                                      \
  -v /dev:/dev                                      \
  --device=/dev/ttyUSB0                             \
  --device=/dev/ttyUSB1                             \
  --device=/dev/ttyUSB2                             \
  --dns 8.8.8.8                                     \
  --name uas-at-ucla_controls                       \
  uas-at-ucla_controls                              \
  bash -c "$DOCKER_BUILD_CMD"

echo "Started uas-at-ucla_controls docker image. Waiting for it to boot..."

# Wait for docker container to start up.
while [ -z $RUNNING_DOCKER_CONTAINERS ]
do
  RUNNING_DOCKER_CONTAINERS=$(docker ps \
    --filter name=uas-at-ucla_controls \
    --filter status=running \
    --format "{{.ID}}" \
    --latest \
  )

  sleep 0.25
done

# Wait for permission scripts to execute.
./tools/scripts/controls/exec.sh "while [ ! -f /tmp/uas_init ];do sleep 0.25;done"
