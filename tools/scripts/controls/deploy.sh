#!/bin/bash

PORT=9013
USER=uas
HOST="uasatucla.org"
LOCAL_BAZEL_RASPI_OUTPUT_ROOT="tools/cache/bazel/execroot/com_uclauas/bazel-out/raspi-fastbuild/bin"
REMOTE_PATH="/home/$USER/uasatucla_controls"

rsync \
  -rvz \
  -e "ssh -p $PORT" \
  --progress \
  "$LOCAL_BAZEL_RASPI_OUTPUT_ROOT/$1" "$USER@$HOST":"$REMOTE_PATH"

ssh -p $PORT "$USER@$HOST" killall gpio_writer
