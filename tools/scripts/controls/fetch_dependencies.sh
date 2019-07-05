#!/bin/bash

ATTEMPTS=0
FAIL_WAIT=5

while [ $ATTEMPTS -le 30 ]
do
  ((ATTEMPTS++))

  echo "Attempting dependencies fetch..."

  bazel fetch //src/...
  if [ $? -ne 0 ]
  then
    sleep $FAIL_WAIT
    continue
  fi
  bazel fetch //lib/...
  if [ $? -ne 0 ]
  then
    sleep $FAIL_WAIT
    continue
  fi

  break
done
