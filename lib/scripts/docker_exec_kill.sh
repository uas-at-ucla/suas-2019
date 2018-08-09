#!/bin/bash

for f in /tmp/docker-exec-*;do kill $(cat $f);done
