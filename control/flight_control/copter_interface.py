import sys
sys.dont_write_bytecode = True

import dronekit
import time
import websocket
import argparse
import math
import random
import thread
import threading
import signal
from datetime import datetime
from pymavlink import mavutil

class Sensor:
    def set(self, value):
        if value is None:
            return

        print("SETTING: " + str(value))
        self.last_update = time.time()
        self.value = value

class Sensors:
    def __init__(self):
        self.telemetry = {
                "state": Sensor(),
                "armed": Sensor(),
                "voltage": Sensor(),
                "last_heartbeat": Sensor(),
                "gps_lat": Sensor(),
                "gps_lng": Sensor(),
                "gps_alt": Sensor(),
                "gps_rel_alt": Sensor(),
                "gps_satellites": Sensor(),
                "velocity_x": Sensor(),
                "velocity_y": Sensor(),
                "velocity_z": Sensor(),
                "pitch": Sensor(),
                "roll": Sensor(),
                "yaw": Sensor(),
                "heading": Sensor(),
                "ground_speed": Sensor(),
                "air_speed": Sensor()
        }

    def set(self, vehicle):
        self.time = time.time()

        self.telemetry["state"].set(vehicle.system_status.state)
        self.telemetry["armed"].set(vehicle.armed)
        self.telemetry["voltage"].set(self.cut_from_string(str( \
                vehicle.battery), "Battery:voltage="))
#       self.last_heartbeat = int(vehicle.last_heartbeat)

#       self.gps_lat = float(self.cut_from_string(str( \
#                       vehicle.location.global_frame), "lat="))
#       self.gps_lng = float(self.cut_from_string(str( \
#                       vehicle.location.global_frame), "lon="))
#       self.gps_alt = float(self.cut_from_string(str( \
#                       vehicle.location.global_frame), "alt="))
#       self.gps_rel_alt = float(self.cut_from_string(str( \
#                       vehicle.location.global_relative_frame), \
#                       "alt="))
#       self.gps_satellites = int(self.cut_from_string(str( \
#                       vehicle.gps_0), "num_sat="))
#       self.velocity_x = vehicle.velocity[0]
#       self.velocity_y = vehicle.velocity[1]
#       self.velocity_z = vehicle.velocity[2]
#       self.roll = float(self.cut_from_string(str( \
#                       vehicle.attitude), "roll="))
#       self.set_if_not_none(self.cut_from_string(str( \
#                       vehicle.attitude), "roll="), self.roll, float)
#       print(self.roll)
#       self.pitch = float(self.cut_from_string(str( \
#                       vehicle.attitude), "pitch="))
#       self.yaw = float(self.cut_from_string(str( \
#                       vehicle.attitude), "yaw="))
#       self.heading = int(vehicle.heading)
#       self.ground_speed = float(vehicle.groundspeed)
#       self.air_speed = float(vehicle.airspeed)

    def set_if_not_none(self, value, destination, destination_type):
        if value == None:
            return
        print("PASS!")

        destination = destination_type(value)

    def cut_from_string(self, raw, key):
        start = raw.find(key) + len(key)
        end = raw.find(",", start)
        if end is -1:
            end = len(raw)

        return_str = raw[start:end]
        if str(return_str) == "None" or str(return_str) == "":
            return None
        else:
            return return_str

class SensorReader:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.sensors = Sensors()

        thread.start_new_thread(self.read_telemetry, ())

    def read_telemetry(self):
        while True:
            print("run!")
            self.sensors.set(self.vehicle)
            time.sleep(0.1)

def connect_to_drone(address):
    # Initializes a Dronekit instance to interface with the flight controller
    # and returns this instance.

    print("Connecting to drone at " + address)

    vehicle = dronekit.connect(ip = address, \
                               baud = 115200)

    vehicle.wait_ready("autopilot_version")

    print("Connected to drone")

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

    sensor_reader = SensorReader(vehicle)

    signal.pause()

if __name__ == "__main__":
    main()
