#!/bin/bash

pigpiod

cd /home/pi/drone_code_deploy/scripts
su -c "./start_drone_code.sh" pi
