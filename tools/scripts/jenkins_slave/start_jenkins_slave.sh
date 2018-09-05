#!/bin/bash

JENKINS_SLAVE_TAG=uas-at-ucla_jenkins-slave
CRED_ID=$1

docker build -t uas-at-ucla_jenkins-slave tools/dockerfiles/jenkins_slave

UAS_AT_UCLA_ENV_DOCKER_RUNNING_CONTAINER=$(docker ps \
  --filter name=$JENKINS_SLAVE_TAG \
  --filter status=running \
  --format "{{.ID}}" \
  --latest \
  )

if [ ! -z $UAS_AT_UCLA_ENV_DOCKER_RUNNING_CONTAINER ]
then
  echo "KILLING $UAS_AT_UCLA_ENV_DOCKER_RUNNING_CONTAINER"
  docker kill $UAS_AT_UCLA_ENV_DOCKER_RUNNING_CONTAINER
fi

UAS_AT_UCLA_ENV_DOCKER_CONTAINER=$(docker ps \
  --filter name=$JENKINS_SLAVE_TAG \
  --format "{{.ID}}" \
  --latest
  )

if [ ! -z $UAS_AT_UCLA_ENV_DOCKER_CONTAINER ]
then
  echo "Removing old container with ID $UAS_AT_UCLA_ENV_DOCKER_CONTAINER"
  docker rm $UAS_AT_UCLA_ENV_DOCKER_CONTAINER
fi

DOCKER_GROUP_ID=$(getent group docker | awk -F: '{printf "%d\n", $3}')

docker run \
  -it \
  --rm \
  --name $JENKINS_SLAVE_TAG \
  -v ~/.ssh:/home/jenkins_uasatucla/.ssh \
  -v $(pwd)/tools/scripts/jenkins_slave:/home/jenkins_uasatucla/scripts \
  -v $(pwd)/tools/cache/jenkins_slave:/home/jenkins_uasatucla/slave \
  -v /var/run/docker.sock:/var/run/docker.sock \
  $JENKINS_SLAVE_TAG \
  bash -c "
  getent group $(id -g) || groupadd -g $(id -g) host_group;
  usermod -u $(id -u) -g $(id -g) jenkins_uasatucla;
  chown -R jenkins_uasatucla:jenkins_uasatucla /home/jenkins_uasatucla;
  service ssh start
  groupmod -g $DOCKER_GROUP_ID docker
  su - jenkins_uasatucla bash -c \"
  PORT=9000
  while true
  do
    nc -z uasatucla.org \\\$PORT
    echo \\\"TRYING \\\$PORT\\\"
    if [[ \\\$? == 0 ]]
    then
      break
    fi
    PORT=\\\$((PORT + 1))
    sleep 1
  done
  echo \\\"USING PORT \\\$PORT\\\"
  /home/jenkins_uasatucla/scripts/start_ssh_tunnel.sh \\\$PORT &
  /home/jenkins_uasatucla/scripts/create_jenkins_slave.sh $(hostname) \\\$PORT $CRED_ID $(pwd)
  sleep infinity\""
