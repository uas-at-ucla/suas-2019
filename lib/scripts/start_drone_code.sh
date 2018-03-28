#!/bin/bash

# Clean up previously running code.
killall core
killall flight_loop
killall io
killall ground_communicator
ipcrm --all

# Start code in background.
screen -d -m /home/pi/suas_2018_deploy/core &
screen -d -m /home/pi/suas_2018_deploy/flight_loop &
screen -d -m /home/pi/suas_2018_deploy/io &
screen -d -m /home/pi/suas_2018_deploy/ground_communicator &
