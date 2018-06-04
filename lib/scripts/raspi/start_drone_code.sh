#!/bin/bash

# Clean up previously running code.
killall aos_core
killall flight_loop
killall io
killall ground_communicator
killall log_writer
killall screen
killall mavproxy.py
killall gphotofs
killall gphoto
killall tag_photos.sh

# Clean up shared memory when restarting AOS core message queues.
ipcrm --all

# Start code in background.
screen -d -m /home/pi/suas_2018_deploy/executables/aos_core &
sleep 0.25;
screen -d -m /home/pi/suas_2018_deploy/scripts/run_mavproxy.sh &
screen -d -m /home/pi/suas_2018_deploy/executables/flight_loop &
screen -d -m /home/pi/suas_2018_deploy/executables/io &
screen -d -m /home/pi/suas_2018_deploy/executables/ground_communicator &
screen -d -m /home/pi/suas_2018_deploy/executables/log_writer &
screen -d -m /home/pi/suas_2018_deploy/scripts/tag_photos.sh &
