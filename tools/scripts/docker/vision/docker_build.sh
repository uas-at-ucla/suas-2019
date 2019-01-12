#!/bin/bash
cd $(dirname $0)
if [ $(uname -s) == "Darwin" ]
then
    source ../start_machine_mac.sh
fi
docker build -f ../../../dockerfiles/vision/Dockerfile -t uas-at-ucla_vision ../../../../

