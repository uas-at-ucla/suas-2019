#!/bin/bash

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
  # Docker environment already running, so no need to start it.
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
docker build -t uas-at-ucla_software tools/docker

if [ $? -ne 0 ]; then
    echo "Error building UAS env docker container."
    exit 1
fi

# Create network for docker container to use.
docker network create -d bridge uas_bridge > /dev/null || true

# Start docker container and let it run forever.
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
