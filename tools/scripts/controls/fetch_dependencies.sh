#!/bin/bash

ATTEMPTS=0
FAIL_WAIT=5

BAZEL_FETCH_FLAGS="--repository_cache=$(pwd)/tools/cache/downloaded "
if [ -n "$CONTINUOUS_INTEGRATION" ]
then
  # Don't do parallel downloads in CI since it is unreliable on some machines.
  BAZEL_FETCH_FLAGS="$BAZEL_FETCH_FLAGS --loading_phase_threads=1 --noshow_progress "
fi

if [ -n "$JENKINS_URL" ] && [ -n "$JENKINS_NODE_COOKIE" ]
then
  echo "Caching to jenkins slave!"
  BAZEL_FETCH_FLAGS="$BAZEL_FETCH_FLAGS --repository_cache=/tmp/bazel_downloaded "
fi
  

while [ $ATTEMPTS -le 5 ]
do
  ((ATTEMPTS++))

  echo "Attempting dependencies fetch..."

  bazel fetch $BAZEL_FETCH_FLAGS //tools/cpp/... //src/... //lib/...
  if [ $? -ne 0 ]
  then
    sleep $FAIL_WAIT
    continue
  fi

  break
done
