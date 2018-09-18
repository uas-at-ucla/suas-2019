#!/bin/bash

if [ $(uname -s) == "Darwin" ]
then
  docker version > /dev/null 2>&1
  if [ $? -ne 0 ]
  then
    UAS_ENV_MACHINE_STATUS="$(docker-machine status uas-env 2> /dev/null)"
    if [ $? -ne 0 ]
    then
      echo ""
      echo "Creating uas-env docker machine:"
      DOCKER_MACHINE_CREATE_CMD="docker-machine create -d virtualbox --virtualbox-memory 4096 --virtualbox-cpu-count 4 --virtualbox-disk-size 8000 uas-env"
      echo "$DOCKER_MACHINE_CREATE_CMD"
      eval "$DOCKER_MACHINE_CREATE_CMD"
      docker-machine-nfs uas-env --shared-folder=$(pwd) -f --nfs-config="-alldirs -mapall=$(id -u):20"
      echo ""
      echo "Started uas-env docker machine. To stop it, run \"docker-machine stop uas-env\""
      echo ""
    else
      if [ "$UAS_ENV_MACHINE_STATUS" != "Running" ]
      then
        echo ""
        echo "Starting uas-env docker machine. To stop it, run \"docker-machine stop uas-env\""
        echo ""
        docker-machine start uas-env
      fi
    fi
    eval $(docker-machine env uas-env)
  fi
fi
