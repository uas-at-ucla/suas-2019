#!/bin/bash

ATTEMPTS=0
MAX_ATTEMPTS=5
FAIL_WAIT=5

BAZEL_FETCH_FLAGS=""
if [ -n "$CONTINUOUS_INTEGRATION" ]
then
  # Don't do parallel downloads in CI since it is unreliable on some machines.
  BAZEL_FETCH_FLAGS="$BAZEL_FETCH_FLAGS --loading_phase_threads=1 --noshow_progress "
  BAZEL_FETCH_FLAGS="$BAZEL_FETCH_FLAGS --repository_cache=/tmp/uasatucla/bazel_downloaded "
fi

while [ $ATTEMPTS -le 5 ]
do
  echo "Attempting dependencies fetch..."

  bazel fetch $BAZEL_FETCH_FLAGS //tools/cpp/... //src/... //lib/...
  if [ $? -ne 0 ]
  then
    if [ $ATTEMPTS -ge $MAX_ATTEMPTS ]
    then
      exit 1
    fi

    sleep $FAIL_WAIT

    ((ATTEMPTS++))
    continue
  fi

  break
done

