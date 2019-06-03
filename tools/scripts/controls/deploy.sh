#!/bin/bash

PORT=22
USER=uas
HOST="192.168.1.21"
LOCAL_BAZEL_RASPI_OUTPUT_ROOT="tools/cache/bazel/execroot/com_uclauas/bazel-out/raspi-fastbuild/bin"
REMOTE_PATH="/home/$USER/uasatucla_controls"

rsync \
  -rvz \
  -e "ssh -p $PORT" \
  --progress \
  "$LOCAL_BAZEL_RASPI_OUTPUT_ROOT/$1" "$USER@$HOST":"$REMOTE_PATH"

ssh -p $PORT "$USER@$HOST" killall $2 || true
