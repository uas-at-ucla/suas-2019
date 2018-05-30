#!/bin/bash

pigpiod

cd /home/pi/suas_2018_deploy/scripts
su -c "./start_drone_code.sh" pi
