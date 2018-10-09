#!/bin/bash
cd $(dirname $0)
source ../start_machine_mac.sh
cd ../../../../src/vision

if [[ $(which wslpath 2>/dev/null) = "/bin/wslpath" ]]
then
    true_path="$(wslpath -w $(pwd))\\data_local"
else
    true_path="$(pwd)/data_local"
fi

docker run \
    --mount type=bind,source="$true_path",target=/suas/src/vision/data_local \
    uas-at-ucla_vision:latest "$@"
