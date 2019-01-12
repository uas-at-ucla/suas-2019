#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

unset INTEROP_DOCKER_RUNNING_CONTAINER
unset INTEROP_DOCKER_CONTAINER

INTEROP_DOCKER_RUNNING_CONTAINER=$(docker ps                                   \
  --filter name=interop-server                                                 \
  --filter status=running                                                      \
  --format "{{.ID}}"                                                           \
  --latest                                                                     \
)

if [ ! -z $INTEROP_DOCKER_RUNNING_CONTAINER ]
then
  echo "Interop Docker container already running."

  exit
fi

INTEROP_DOCKER_CONTAINER=$(docker ps                                           \
  --filter name=uas-at-ucla_interop-server                                     \
  --format "{{.ID}}"                                                           \
  --latest                                                                     \
)

if [ ! -z $INTEROP_DOCKER_CONTAINER ]
then
  echo "Removing old container with ID $INTEROP_DOCKER_CONTAINER"
  docker rm $INTEROP_DOCKER_CONTAINER
fi

echo ""
echo "Starting Interop Server at http://localhost:8000"
echo "Username: testadmin"
echo "Password: testpass"
echo ""

# Start docker container and let it run forever.
docker run --rm -t -p 8000:80 --name uas-at-ucla_interop-server auvsisuas/interop-server
