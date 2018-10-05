#!/bin/bash

source tools/scripts/docker/start_machine_mac.sh

unset UAS_AT_UCLA_IMAGE
UAS_AT_UCLA_IMAGE=$(docker ps \
  --filter status=running \
  --filter name=uas-at-ucla_ground \
  --format "{{.ID}}" \
  --latest)

if [ -z $UAS_AT_UCLA_IMAGE ]
then
  exit 1
fi

docker exec -t $UAS_AT_UCLA_IMAGE sh -c "for f in /tmp/docker-exec-*.pid;do PID=\$(cat \"\$f\");echo \"KILLING \$PID\";kill -15 \$PID > /dev/null 2>&1 || true;rm \$f;done"
