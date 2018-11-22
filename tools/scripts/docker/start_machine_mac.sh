#!/bin/bash

if [ "$EUID" == 0 ]
  then echo "Don't run as root."
  exit 1
fi

if [ $(uname -s) == "Darwin" ]
then
  docker version > /dev/null 2>&1
  if [ $? -ne 0 ]
  then
    UAS_ENV_MACHINE_STATUS="$(docker-machine status uas-env 2> /dev/null)"
    if [ $? -ne 0 ]
    then
      # Script must be able to ask for sudo privileges to run properly.
      if [ ! -t 1 ]
      then
        echo "Run ./tools/scripts/docker/start_machine_mac.sh to start docker VM."
        exit 1
      fi

      echo ""
      echo "Creating uas-env docker machine:"
      DOCKER_MACHINE_CREATE_CMD="docker-machine create -d virtualbox --virtualbox-memory 3072 --virtualbox-cpu-count 2 --virtualbox-disk-size 16000 uas-env"
      echo "$DOCKER_MACHINE_CREATE_CMD"
      eval "$DOCKER_MACHINE_CREATE_CMD"
      VBoxManage controlvm "uas-env" natpf1 "ground_ui,tcp,,3000,,3000"
      VBoxManage controlvm "uas-env" natpf1 "ground_server,tcp,,8081,,8081"
      VBoxManage controlvm "uas-env" natpf1 "interop,tcp,,8000,,8000"
      VBoxManage controlvm "uas-env" natpf1 "vision_server,tcp,,8099,,8099"
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
else
  echo "This script should only be run on MacOS."
fi
