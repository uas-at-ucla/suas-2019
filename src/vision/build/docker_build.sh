#!/bin/bash
cd $(dirname $0)
docker build -f Dockerfile -t suas_vision ../../../
