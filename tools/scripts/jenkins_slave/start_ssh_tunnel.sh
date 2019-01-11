#!/bin/bash

echo "Starting ssh tunnel..."

while true
do
  ssh -N -R "\*:$1:localhost:22" uas@uasatucla.org
  sleep 1
  echo "Reconnecting SSH tunnel..."
done
