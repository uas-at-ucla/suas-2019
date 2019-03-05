#!/bin/bash

while true
do
    roslaunch mavros px4.launch fcu_url:="udp://:8084@0.0.0.0:8084"
    sleep 1
done