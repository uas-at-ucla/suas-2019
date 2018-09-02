#!/bin/bash

mkdir -p tools/cache/jenkins_server

docker build -t uasatucla_jenkins tools/dockerfiles/jenkins_server

# Start Jenkins master.
docker run -d -p 8085:8080 \
  -p 50000:50000 \
  -v ./tools/cache/jenkins_server:/var/jenkins_home \
  -v /var/run/docker.sock:/var/run/docker.sock \
  uasatucla_jenkins
