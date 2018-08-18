#!/bin/bash

UAS_ENV_DOCKER_RUNNING_CONTAINER=$(docker ps \
  --filter name=uas_env \
  --filter status=running \
  --format "{{.ID}}" --latest\
  )

if [ ! -z $UAS_ENV_DOCKER_RUNNING_CONTAINER ]
then
  echo "UAS@UCLA docker environment already running!"
  echo "Proceeding with existing docker container."
  exit
fi

UAS_ENV_DOCKER_CONTAINER=$(docker ps \
  --filter name=uas_env \
  --format "{{.ID}}" \
  --latest
  )

if [ ! -z $UAS_ENV_DOCKER_CONTAINER ]
then
  echo "Removing old container with ID $UAS_ENV_DOCKER_CONTAINER"
  docker rm $UAS_ENV_DOCKER_CONTAINER
fi


# Build docker container.
echo "Building UAS@UCLA docker container."
docker build -t uas-at-ucla_software tools/docker

# Create network for docker container to use.
echo "Setting up network..."
docker network create -d bridge uas_bridge > /dev/null || true

# Start docker container and let it run forever.
echo "Running container..."
docker run \
  -it \
  -d \
  --rm \
  --net uas_bridge \
  -v $(pwd):/home/uas/code_env/ \
  -v $(pwd)/tools/docker/cache/bazel:/home/uas/.cache/bazel/_bazel_uas  \
  --name uas_env \
  --env=LOCAL_USER_ID="$(id -u)" \
  uas-at-ucla_software \
  "bazel | sleep infinity"

# Wait for docker container to start up.
echo "Waiting for container to boot..."
unset UAS_AT_UCLA_RUNNING_DOCKER_CONTAINERS
while [ -z $UAS_AT_UCLA_RUNNING_DOCKER_CONTAINERS ]
do
  UAS_AT_UCLA_RUNNING_DOCKER_CONTAINERS=$(docker ps \
    --filter name=uas_env \
    --format "{{.ID}}" \
    --latest \
  )

  sleep 0.25
done
