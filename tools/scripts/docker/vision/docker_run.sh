#!/bin/bash
cd $(dirname $0)
cd ../../../../src/vision
docker run \
    --mount type=bind,source="$(pwd)"/data_local,target=/suas/src/vision/data_local \
    suas_vision:latest "$@"

