#!/bin/bash
cd $(dirname $0)
docker build -f ../../../dockerfiles/vision/Dockerfile -t suas_vision ../../../../

