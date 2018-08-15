#!/bin/bash

function docker_exec {
    unset UAS_AT_UCLA_IMAGE
    UAS_AT_UCLA_IMAGE=$(docker ps \
      --filter status=running \
      --filter name=uas_env \
      --format "{{.ID}}" \
      --latest)

    PIDFILE=/tmp/docker-exec-$$
    NAMEFILE=/tmp/docker-exec-$$

    docker exec -t $UAS_AT_UCLA_IMAGE \
      bash -c "echo \"\$\$\" > \"$PIDFILE\".pid; echo \"$*\" > \"$NAMEFILE\".name;$*"

    PID=$!
}

docker_exec "$@"
