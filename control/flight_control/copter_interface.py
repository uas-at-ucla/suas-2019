import dronekit
import time
import websocket
import sys
import argparse
import math
import random
import threading
import signal
from datetime import datetime
from pymavlink import mavutil

sys.dont_write_bytecode = True

def connect_to_drone(address):
    print "Connecting to drone..."
    vehicle = dronekit.connect(ip = address, \
                               baud = 115200)
    vehicle.wait_ready("autopilot_version")

    print "CONNECTED!"
    return vehicle

def main():
    parser = argparse.ArgumentParser( \
            description = "Interface with flight controller.")

    parser.add_argument("--address", \
                        type = str, \
                        help = "address")

    args = parser.parse_args()

    if args.address is None:
        parser.print_help()
        return

    vehicle = connect_to_drone(args.address)

    signal.pause()

if __name__ == "__main__":
    main()
