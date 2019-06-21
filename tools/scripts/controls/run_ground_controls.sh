#!/bin/bash

# For now, this is a copy of run_env.sh, but it runs without the Docker ip network so that it doesn't clash with the physical network at competition

source tools/scripts/docker/start_machine_mac.sh

unset ENV_DOCKER_RUNNING_CONTAINER
unset ENV_DOCKER_CONTAINER
unset RUNNING_DOCKER_CONTAINERS

# Check if Dockerfile was updated, indicating that the existing docker container
# needs to be killed.
mkdir -p tools/cache/checksums
CHECKSUM=$(md5sum tools/dockerfiles/controls/Dockerfile)
MATCH=0

CONTAINER_NAME=uas-at-ucla_ground-controls

function get_running_container {
  echo $(docker ps                                                             \
    --filter name=$CONTAINER_NAME                                              \
    --filter status=running                                                    \
    --format "{{.ID}}"                                                         \
    --latest                                                                   \
  )
}

function interrupt_exec {
  docker kill $CONTAINER_NAME > /dev/null 2>&1

  RUNNING_DOCKER_CONTAINER=$(get_running_container)
  while [ ! -z $RUNNING_DOCKER_CONTAINER ]
  do
    docker kill $CONTAINER_NAME > /dev/null 2>&1

    RUNNING_DOCKER_CONTAINER=$(get_running_container)
    sleep 0.5
  done

  exit
}

trap interrupt_exec INT

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
    --filter name=uas-at-ucla_ground-controls \
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
        --filter uas-at-ucla_ground-controls \
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
  --filter name=uas-at-ucla_ground-controls \
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
  --filter name=uas-at-ucla_ground-controls \
  --format "{{.ID}}" \
  --latest
  )

if [ ! -z $ENV_DOCKER_CONTAINER ]
then
  echo "Removing old container with ID $ENV_DOCKER_CONTAINER"
  docker rm $ENV_DOCKER_CONTAINER
fi

BUILD_FLAGS="-t uas-at-ucla_controls"

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

mkdir -p tools/cache/bazel
pwd

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
  sudo -u uas bash -c \" \
  source /home/uas/.bashrc; \
  export ROS_MASTER_URI=http://192.168.1.20:11311; \
  export ROS_IP=192.168.1.21; \
  env; \
  bash\""
  # bazel run //src/controls/ground_controls:ground_controls 192.168.1.21\""

docker run                                    \
  -it                                          \
  --rm                                        \
  --cap-add=SYS_PTRACE                        \
  --security-opt seccomp=unconfined           \
  --network host                              \
  --privileged                      \
  -v /dev:/dev                      \
  --device=/dev/ttyUSB0             \
  --device=/dev/ttyUSB1             \
  --device=/dev/ttyUSB2             \
  -v $ROOT_PATH:/home/uas/code_env            \
  -v ~/.ssh:/home/uas/.ssh                    \
  -e DISPLAY=$DISPLAY                         \
  -e ROS_MASTER_URI=http://192.168.1.20:11311 \
  -e ROS_IP=192.168.1.10                      \
  -v /tmp/.X11-unix:/tmp/.X11-unix            \
  --privileged                                \
  --dns 8.8.8.8                               \
  --name uas-at-ucla_ground-controls          \
  uas-at-ucla_controls                        \
  bash -c "$DOCKER_BUILD_CMD"

# Wait for V-REP container to initialize.
unset RUNNING_DOCKER_CONTAINERS
while [ -z $RUNNING_DOCKER_CONTAINER ]
do
  RUNNING_DOCKER_CONTAINER=$(get_running_vrep_container)
  sleep 0.25
done

echo "Started docker container"

# Wait for V-REP container to exit.
while [ ! -z $RUNNING_VREP_DOCKER_CONTAINER ]
do
  RUNNING_DOCKER_CONTAINER=$(get_running_vrep_container)
  sleep 0.5
done
