#!/bin/bash

mavlink-routerd
  -t 0
  -e 127.0.0.1:8084
  -e 192.168.1.3:8085
  /dev/ttyS0:921600
  2>&1

