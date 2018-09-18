#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

unset INTEROP_DOCKER_RUNNING_CONTAINER
unset INTEROP_DOCKER_CONTAINER

INTEROP_DOCKER_RUNNING_CONTAINER=$(docker ps \
  --filter name=interop-server \
  --filter status=running \
  --format "{{.ID}}" \
  --latest \
  )

if [ ! -z $INTEROP_DOCKER_RUNNING_CONTAINER ]
then
  # Kill running docker simulators.
  docker kill $INTEROP_DOCKER_RUNNING_CONTAINER
fi

INTEROP_DOCKER_CONTAINER=$(docker ps \
  --filter name=interop-server \
  --format "{{.ID}}" \
  --latest
  )

if [ ! -z $INTEROP_DOCKER_CONTAINER ]
then
  echo "Removing old container with ID $INTEROP_DOCKER_CONTAINER"
  docker rm $INTEROP_DOCKER_CONTAINER
fi

# Start docker container and let it run forever.
docker run -d --restart=unless-stopped --interactive --tty --publish 8000:80 --name interop-server auvsisuas/interop-server

echo ""
VM_IP=$(docker-machine ip uas-env 2> /dev/null)
if [ $? -eq 0 ]
then
  echo "Starting Interop Server at http://$VM_IP:8000 (this is the IP of the uas-env docker-machine)"
else
  echo "Starting Interop Server at http://localhost:8000"
fi
echo ""

# Poll server up to 5 min for healthiness before proceeding.
echo "Waiting for setup to complete..."
for i in {1..300};
do
    docker inspect -f "{{.State.Health.Status}}" interop-server | grep "^healthy$" && exit 0 || sleep 1;
done

exit 1
