#!/bin/bash
cd $(dirname $0)
source ../start_machine_mac.sh
cd ../../../../src/vision

# docker running on windows expects a windows-style path
if [[ $(which wslpath 2>/dev/null) = "/bin/wslpath" ]]
then
    true_path="$(wslpath -w $(pwd))\\data_local"
else
    true_path="$(pwd)/data_local"
fi

if [[ $1 = "client" ]]
then
    extra_parameters='--network=host' # allows it to connect to localhost
else
    extra_parameters="-p 8099:8099" # allows clients to connect
fi

# generate a random id
vision_container_id=$(head -c 5 /dev/urandom | base32)

docker run \
    $extra_parameters \
    --name uas-at-ucla_vision-$vision_container_id \
    --mount type=bind,source="$true_path",target=/suas/src/vision/data_local \
    uas-at-ucla_vision:latest "$@"
