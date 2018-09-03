#!/bin/bash

mkdir -p ./tools/cache/jenkins-server

docker build -t uasatucla_jenkins ./tools/dockerfiles/jenkins-server

# Start Jenkins master.
docker run \
  -i \
  -p 8085:8080 \
  -p 50000:50000 \
  -v $(pwd)/tools/cache/jenkins-server:/var/jenkins_home \
  -v /var/run/docker.sock:/var/run/docker.sock \
  uasatucla_jenkins
