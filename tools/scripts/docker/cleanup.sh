#!/bin/bash

if [ $(uname -s) == "Darwin" ]
then
  UAS_ENV_MACHINE_STATUS="$(docker-machine status uas-env 2> /dev/null)"
  if [ "$UAS_ENV_MACHINE_STATUS" != "Running" ]
  then
    echo "Docker is not running"
    exit
  fi
  eval $(docker-machine env uas-env 2> /dev/null)
fi

docker version > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo "Docker is not running"
  exit
fi

echo -n "Remove all containers (y/n)? "
read ANSWER
if [ "$ANSWER" != "${ANSWER#[Yy]}" ]
then
  RUNNING_CONTAINERS=$(docker ps -q)
  if [ ! -z $RUNNING_CONTAINERS ]
  then
    docker kill $RUNNING_CONTAINERS 2> /dev/null
  fi
  ALL_CONTAINERS=$(docker ps -a -q)
  if [ ! -z $ALL_CONTAINERS ]
  then
    docker rm $ALL_CONTAINERS 2> /dev/null
  fi
  DANGLING_IMAGES=$(docker images -q -f dangling=true)
  if [ ! -z $DANGLING_IMAGES ]
  then
    echo ""
    echo "Deleting dangling/untagged images..."
    docker rmi $DANGLING_IMAGES 2> /dev/null
  fi
else
  # get all running docker container names
  CONTAINERS=$(docker ps | awk '{if(NR>1) print $NF}')

  # loop through all containers
  for CONTAINER in $CONTAINERS
  do
    echo -n "Kill and remove $CONTAINER container (y/n)? "
    read ANSWER
    if [ "$ANSWER" != "${ANSWER#[Yy]}" ]
    then
      docker kill $CONTAINER
      docker rm $CONTAINER 2> /dev/null
    fi
  done
fi

if [ $(uname -s) == "Darwin" ]
then
  # prompt to stop virtual machine
  echo ""
  MACHINES=$(docker-machine ls --filter state=Running | awk '{if(NR>1) print $1}')
  for MACHINE in $MACHINES
  do
    echo -n "Stop $MACHINE docker machine (this will stop any remaining containers) (y/n)? "
    read ANSWER
    if [ "$ANSWER" != "${ANSWER#[Yy]}" ]
    then
      docker-machine stop $MACHINE
    fi
  done
fi
