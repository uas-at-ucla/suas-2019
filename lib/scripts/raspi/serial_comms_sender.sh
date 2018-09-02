#!/bin/bash

PORT=0

while true;do
  if [ $PORT = 3 ];then
    PORT=0
  fi

  echo "TRYING USB$PORT"

  # Do this in a loop in case we can't find the serial device.
  python /home/pi/drone_code_deploy/scripts/serial_comms/serial_comms.py sender --device /dev/ttyUSB$PORT;
  let PORT="$PORT+1"
  sleep 0.3;
done

