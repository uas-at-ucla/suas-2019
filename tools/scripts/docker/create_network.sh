#!/bin/bash
docker network create -d bridge --subnet 192.168.2.0/24 --gateway 192.168.2.1 uas_bridge
