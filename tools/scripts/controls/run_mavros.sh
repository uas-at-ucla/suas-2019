#!/bin/bash

while true
do
    roslaunch mavros px4.launch fcu_url:="udp://:9011@0.0.0.0:9011"
    sleep 1
done
