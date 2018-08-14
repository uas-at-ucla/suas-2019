#!/bin/bash

function docker_exec {
    IMAGE=$1
    PIDFILE=/tmp/docker-exec-$$
    NAMEFILE=/tmp/docker-exec-$$-name
    shift
    docker exec $IMAGE bash -c "echo \"\$\$\" > $PIDFILE; echo \"$*\" > \"$NAMEFILE\"_name;$*" &
    PID=$!
    rm $PIDFILE
}

docker_exec "$@"
