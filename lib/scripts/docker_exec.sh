#!/bin/bash

function docker_cleanup {
    docker exec $IMAGE bash -c "if [ -f $PIDFILE ]; then kill -TERM -\$(cat $PIDFILE); rm $PIDFILE; fi"
}

function docker_exec {
    IMAGE=$1
    PIDFILE=/tmp/docker-exec-$$
    shift
    #trap 'kill $PID; docker_cleanup $IMAGE $PIDFILE' TERM INT
    docker exec $IMAGE bash -c "echo \"\$\$\" > $PIDFILE; $*" &
    PID=$!
    #wait $PID
    #trap - TERM INT HUP EXIT
    #wait $PID
}

docker_exec "$@"
