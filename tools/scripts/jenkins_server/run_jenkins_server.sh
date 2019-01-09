#!/bin/bash

mkdir -p ./tools/cache/jenkins_server

docker build -t uas-at-ucla_jenkins ./tools/dockerfiles/jenkins_server

# Start Jenkins master.
docker run \
  -i \
  -p 8085:8080 \
  -p 50000:50000 \
  -v $(pwd)/tools/cache/jenkins_server:/var/jenkins_home \
  -v /var/run/docker.sock:/var/run/docker.sock \
  uas-at-ucla_jenkins
