#!/bin/bash

# Clean up previously running code.
killall flight_loop
killall io
killall ground_communicator
killall log_writer
killall screen
killall mavproxy.py
killall gphotofs
killall gphoto
killall tag_photos.sh
killall screen

# Start code in background.
screen -d -m /home/pi/drone_code_deploy/scripts/serial_comms/serial_comms_sender.sh &
sleep 3.0;
screen -d -m /home/pi/drone_code_deploy/scripts/run_mavproxy.sh &
screen -d -m /home/pi/drone_code_deploy/executables/flight_loop &
screen -d -m /home/pi/drone_code_deploy/executables/io &
screen -d -m /home/pi/drone_code_deploy/executables/ground_communicator &
screen -d -m /home/pi/drone_code_deploy/executables/log_writer &
screen -d -m /home/pi/drone_code_deploy/scripts/tag_photos.sh &
