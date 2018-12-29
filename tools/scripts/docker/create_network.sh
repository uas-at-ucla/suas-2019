#!/bin/bash
CREATE_NETWORK="false"
NETWORK_UP_TO_DATE=$(docker network inspect uas_bridge)
if [ $? -ne 0 ]; then
  CREATE_NETWORK="true"
else
  NETWORK_UP_TO_DATE=$(echo "$NETWORK_UP_TO_DATE" | grep '"Gateway": "192.168.2.1"')
  if [ -z "$NETWORK_UP_TO_DATE" ]; then
    docker network rm uas_bridge
    CREATE_NETWORK="true"
  fi
fi

if [ "$CREATE_NETWORK" = "true" ]; then
  docker network create -d bridge --subnet 192.168.2.0/24 --gateway 192.168.2.1 uas_bridge
fi

