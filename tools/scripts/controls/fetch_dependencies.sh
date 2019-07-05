#!/bin/bash

ATTEMPTS=0
while [ $ATTEMPTS -le 20 ]
do
  echo "Attempting dependencies fetch..."

  sleep 1
  bazel fetch //src/...
  if [ $? -ne 0 ]
  then
    continue
  fi
  bazel fetch //lib/...
  if [ $? -eq 0 ]
  then
    break
  fi
done
