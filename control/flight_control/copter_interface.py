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
    def __init__(self, value_type):
        # Keep track of the value's variable type that will be provided later
        # on.
        self.__value_type = value_type
        self.__value = None
        self.__last_update = None

    def set(self, value):
        # Update the sensor value, and record the time at which it was updated.
        if value is None:
            return

        self.__last_update = time.time()
        self.__value = self.__value_type(value)

    def get(self):
        if self.__last_update < time.time() - 0.2:
            # Return None if our sensor data has gone stale.
            return None

        return self.__value

class Sensors:
    def __init__(self):
        self.__telemetry_lock = threading.Lock()

        with self.__telemetry_lock:
            self.__telemetry = {
                    "state": Sensor(str),
                    "armed": Sensor(str),
                    "voltage": Sensor(float),
                    "last_heartbeat": Sensor(float),
                    "gps_lat": Sensor(float),
                    "gps_lng": Sensor(float),
                    "gps_alt": Sensor(float),
                    "gps_rel_alt": Sensor(float),
                    "gps_satellites": Sensor(int),
                    "velocity_x": Sensor(float),
                    "velocity_y": Sensor(float),
                    "velocity_z": Sensor(float),
                    "pitch": Sensor(float),
                    "roll": Sensor(float),
                    "yaw": Sensor(float),
                    "heading": Sensor(int),
                    "ground_speed": Sensor(float),
                    "air_speed": Sensor(float)
            }

    def set(self, vehicle):
        # Thread-safe method to set telemetry data by providing a Dronekit
        # vehicle instance.
        with self.__telemetry_lock:
            self.__telemetry["state"].set(vehicle.system_status.state)
            self.__telemetry["armed"].set(vehicle.armed)
            self.__telemetry["voltage"].set(self.__cut_from_string(str( \
                    vehicle.battery), "Battery:voltage="))
            self.__telemetry["last_heartbeat"].set(int(vehicle.last_heartbeat))
            self.__telemetry["gps_lat"].set(self.__cut_from_string(str( \
                    vehicle.location.global_frame), "lat="))
            self.__telemetry["gps_lng"].set(self.__cut_from_string(str( \
                    vehicle.location.global_frame), "lon="))
            self.__telemetry["gps_alt"].set(self.__cut_from_string(str( \
                    vehicle.location.global_frame), "alt="))
            self.__telemetry["gps_rel_alt"].set(self.__cut_from_string(str( \
                            vehicle.location.global_relative_frame), \
                            "alt="))
            self.__telemetry["gps_satellites"].set(self.__cut_from_string(str( \
                            vehicle.gps_0), "num_sat="))
            self.__telemetry["velocity_x"].set(vehicle.velocity[0])
            self.__telemetry["velocity_y"].set(vehicle.velocity[1])
            self.__telemetry["velocity_z"].set(vehicle.velocity[2])
            self.__telemetry["pitch"].set(self.__cut_from_string(str( \
                            vehicle.attitude), "pitch="))
            self.__telemetry["roll"].set(self.__cut_from_string(str( \
                            vehicle.attitude), "roll="))
            self.__telemetry["yaw"].set(self.__cut_from_string(str( \
                            vehicle.attitude), "yaw="))
            self.__telemetry["heading"].set(vehicle.heading)
            self.__telemetry["ground_speed"].set(vehicle.groundspeed)
            self.__telemetry["air_speed"].set(vehicle.airspeed)

    def get(self):
        # Returns a thread-safe copy of the telemetry data.
        with self.__telemetry_lock:
            return self.__telemetry.copy()

    def __cut_from_string(self, raw, key):
        # Extract telemetry values from a string by searching for the key as
        # a prefix and copying everything up to the comma, designating the
        # next value.
        start = raw.find(key) + len(key)
        end = raw.find(",", start)
        if end is -1:
            end = len(raw)

        return_str = raw[start:end]

        # Return the python None type if the string returned is None or our
        # telemetry string is empty.
        if str(return_str) == "None" or str(return_str) == "":
            return None

        return return_str

class SensorReader:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.sensors = Sensors()
        self.should_run = True

        self.reading_thread = thread.start_new_thread(self.read_telemetry, ())

    def __del__(self):
        self.stop()

    def stop(self):
        self.should_run = False

    def read_telemetry(self):
        while self.should_run: 
            self.sensors.set(self.vehicle)
            time.sleep(0.1)

class CopterInterface:
    def __init__(self, address):
        self.vehicle = self.__connect_to_drone(address)
        self.sensor_reader = SensorReader(self.vehicle)

    def __del__(self):
        self.stop()

    def stop(self):
        self.sensor_reader.stop()
        self.vehicle.close()

    def __connect_to_drone(self, address):
        # Initializes a Dronekit instance to interface with the flight
        # controller and returns this instance.

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

    drone_interface = CopterInterface(args.address)

    signal.pause()

if __name__ == "__main__":
    main()
