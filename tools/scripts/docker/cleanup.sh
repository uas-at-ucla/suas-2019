#!/bin/bash

if [ $(uname -s) == "Darwin" ]
then
  eval $(docker-machine env uas-env 2> /dev/null)
fi

docker version > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo "Docker is not running"
  exit
fi

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
