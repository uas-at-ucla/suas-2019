#!/bin/bash

PORT=9013
USER=uas
HOST="uasatucla.org"
REMOTE_PATH="/home/$USER/uasatucla_controls"

rsync -rvz -e "ssh -p $PORT" --progress tools/cache/bazel/execroot/com_uclauas/bazel-out/raspi-fastbuild/bin/src/controls/io/gpio_writer/gpio_writer "$USER@$HOST":$REMOTE_PATH
