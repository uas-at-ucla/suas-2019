#!/bin/bash

while true
do
    export ROS_MASTER_URI=http://192.168.2.21:11311
    roslaunch mavros px4.launch fcu_url:="udp://:8084@0.0.0.0:8084"
    sleep 1
done