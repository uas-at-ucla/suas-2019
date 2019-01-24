#!/bin/bash

docker network inspect uas_bridge | grep -o -P ".{0,9}2/16" | sed 's/\/16//g'
