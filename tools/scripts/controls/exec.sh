#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

function interrupt_exec {
  if [ ! -z $PIDFILE ]
  then
    docker exec -t $UAS_AT_UCLA_IMAGE sh -c "PID=\$(cat $PIDFILE);kill -15 \$PID > /dev/null 2>&1 || true;rm -f $PIDFILE;rm -f $NAMEFILE"
    printf "\033[91mINTERRUPTED!\033[0m\n"
  fi
}

function docker_exec {
    unset UAS_AT_UCLA_IMAGE
    UAS_AT_UCLA_IMAGE=$(docker ps \
      --filter status=running \
      --filter name=uas-at-ucla_controls \
      --format "{{.ID}}" \
      --latest)

    if [ -z $UAS_AT_UCLA_IMAGE ]
    then
      echo "Could not find uas env docker image. Exiting..."
      exit 1
    fi

    PIDFILE=/tmp/docker-exec-$$.pid
    NAMEFILE=/tmp/docker-exec-$$.name

    trap interrupt_exec INT

    docker exec -t -u $(id -u):$(id -g) \
       -e COLUMNS="`tput cols`" -e LINES="`tput lines`" \
       $UAS_AT_UCLA_IMAGE \
      bash -c "echo \"\$\$\" > \"$PIDFILE\"; echo \"$*\" > \"$NAMEFILE\";source /home/uas/.bashrc;$*"
    CODE=$?

    docker exec -t $UAS_AT_UCLA_IMAGE sh -c "rm -f $PIDFILE"
    docker exec -t $UAS_AT_UCLA_IMAGE sh -c "rm -f $NAMEFILE"

    exit $CODE
}

docker_exec "$@"
