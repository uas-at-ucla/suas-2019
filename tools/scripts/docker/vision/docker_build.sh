#!/bin/bash
cd $(dirname $0)
source ../start_machine_mac.sh
docker build -f ../../../dockerfiles/vision/Dockerfile -t uas-at-ucla_vision ../../../../

