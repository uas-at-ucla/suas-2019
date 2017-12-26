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
                    "pixhawk_state": Sensor(str),
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

    def set(self, vehicle, controller):
        # Thread-safe method to set telemetry data by providing a Dronekit
        # vehicle instance.
        with self.__telemetry_lock:
            self.__telemetry["state"].set(controller.get_state())
            self.__telemetry["pixhawk_state"].set(vehicle.system_status.state)
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
    def __init__(self, vehicle, controller):
        self.vehicle = vehicle
        self.controller = controller

        self.sensors = Sensors()
        self.should_run = True

        self.communications_socket = None
        self.send_telemetry_frequency = 3

        self.reading_thread = thread.start_new_thread(self.read_telemetry, ())

    def set_communications_socket(self, communications_socket):
        self.communications_socket = communications_socket

    def read_telemetry(self):
        loop_num = 0
        while self.should_run:
            self.sensors.set(self.vehicle, self.controller)
            if (loop_num == self.send_telemetry_frequency):
                loop_num = 0
                self.send_telemetry()
            loop_num += 1
            time.sleep(0.1)

    def send_telemetry(self):
        if (self.communications_socket):
            sensors = self.sensors.get()

            sensors_to_send = dict()

            for key in sensors:
                sensors_to_send[key] = sensors[key].get()

            self.communications_socket.emit('telemetry', sensors_to_send)

    def stop(self):
        self.should_run = False

class Controller:
    def __init__(self, vehicle):
        self.vehicle = vehicle

        self.print_debug("NEW LOG")
        self.vel_lock = threading.Lock()
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.last_velocity = None
        self.state_lock = threading.Lock()
        self.state = "STANDBY"
        self.should_run = True

        self.reading_thread = thread.start_new_thread(self.control_loop, ())

    def stop(self):
        self.should_run = False

    def print_debug(self, message):
        msg = "(controller @ " + str(datetime.now()) + ") " + message
        print(msg)

    def set_state(self, state):
        with self.state_lock:
            state = state.replace('"', '')
            if self.state == "FAILSAFE" or self.state == "THROTTLE_CUT":
                self.print_debug("Failed to set state: In " + self.state \
                        + " mode.")
                return

            self.state = state

    def get_state(self):
        with self.state_lock:
            return self.state

    def phased_loop(self, frequency, last_loop):
        # Maintain a constant phased loop by adjusting sleep based on how much
        # time it took to run the loop function.
        time.sleep(1.0 / frequency - min(1.0 / frequency, \
                                         time.time() - last_loop))
        return time.time()

    def set_velocity(self, velocity_x, velocity_y, velocity_z):
        # Move vehicle in direction based on specified velocity vectors.
        # velocity_x is latitude
        # velocity_y is longitude
        # velocity_z is altitude (positive is down)

        with self.vel_lock:
            self.last_velocity = time.time()

            self.vel_x = velocity_x
            self.vel_y = velocity_y
            self.vel_z = velocity_z

    def send_velocity(self):
        with self.vel_lock:
            velocity_x = self.vel_x
            velocity_y = self.vel_y
            velocity_z = self.vel_z

        cap = 1000
        velocity_x = max(min(cap, velocity_x), -1 * cap)
        velocity_y = max(min(cap, velocity_y), -1 * cap)
        cap = cap * 0.4
        velocity_z = max(min(cap, velocity_z), -1 * cap)

        msg = self.vehicle.message_factory \
                .set_position_target_local_ned_encode(
            0, # time_boot_ms (not used)
            0, 0, # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # 3D velocity
            0, 0, 0, # x, y, z acceleration (not supported yet)
            0, 0)    # yaw, yaw_rate (not supported yet)
        self.vehicle.send_mavlink(msg)

    def terminate_flight(self):
        while True:
            msg = self.vehicle.message_factory.command_long_encode( \
                0, 0,     # target system, target component
                mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION ,
                0,        # confirmation
                1.0,
                0, 0, 0, 0, 0, 0)
#           for servo in range(1, 8):
#               msg = self.vehicle.message_factory.command_long_encode( \
#                   0, 0,     # target system, target component
#                   mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
#                   0,        # confirmation
#                   servo,    # param 1, servo No
#                   1000,     # param 2, pwm
#                   0, 0, 0, 0, 0)
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)


    def control_loop(self):
        TARGET_TAKEOFF_ALT = 3.0

        last_loop = 0

        # Regular loop.
        while self.should_run:
            last_loop = self.phased_loop(10, last_loop)

            new_state = None
            current_state = self.get_state()

            # Log any change in state.

#           if current_state != "STANDBY" and self.check_velocity_control():
#               self.set_state("FAILSAFE")
#               current_state = "FAILSAFE"

            # Drone state machine.
            if current_state == "STANDBY":
                pass
            elif current_state == "ARM":
                self.set_state("ARM WAIT FOR INIT")
            elif current_state == "ARM WAIT FOR INIT":
                if self.vehicle.is_armable:
                    self.set_state("ARMING")
            elif current_state == "ARMING":
                self.vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.vehicle.armed = True
                self.set_state("ARMED CHECK")
            elif current_state == "ARMED CHECK":
                if self.vehicle.armed:
                    self.set_state("ARMED")
            elif current_state == "ARMED":
                # Wait for other drones to also be armed.
                self.vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.vehicle.armed = True
                pass
            elif current_state == "TAKEOFF":
                self.vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.armed = True
                self.set_state("TAKEOFF SEND COMMAND")
            elif current_state == "TAKEOFF SEND COMMAND":
                self.vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.armed = True
                self.vehicle.simple_takeoff(TARGET_TAKEOFF_ALT)
                self.set_state("TAKEOFF WAIT FOR TARGET ALTITUDE")
            elif current_state == "TAKEOFF WAIT FOR TARGET ALTITUDE":
                self.vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.armed = True
                if self.vehicle.location.global_relative_frame.alt \
                        >= TARGET_TAKEOFF_ALT * 0.85:
                    self.set_state("TAKEN OFF")
                else:
                    pass
                    #self.print_debug("Altitude: " +
                    #    str(self.vehicle.location.global_relative_frame.alt))
            elif current_state == "TAKEN OFF":
                # Wait for other drones to take off.
                self.vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.armed = True
                pass
            elif current_state == "VELOCITY CONTROL":
                self.vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.vehicle.armed = True
                self.send_velocity()
            elif current_state == "LAND":
                # Command vehicle to land.
                self.vehicle.mode = dronekit.VehicleMode("LAND")
                self.set_state("LANDING")
            elif current_state == "LANDING":
                # Wait until vehicle disarms after landing.
                if self.vehicle.armed:
                    pass
                    #self.print_debug("Altitude: " +
                    #    str(self.vehicle.location.global_relative_frame.alt))
                else:
                    self.set_state("LANDED")
            elif current_state == "LANDED":
                # Wait for all other drones to land.
                pass
            elif current_state == "FAILSAFE":
                # Exit control loop and go into failsafe mode.
                break
            elif current_state == "THROTTLE_CUT":
                # Exit control loop and go into throttle cut mode.
                break
            else:
                self.print_debug("UNKNOWN CONTROLLER STATE: " + current_state)
                self.set_state("FAILSAFE")

        if current_state == "THROTTLE_CUT":
            self.terminate_flight()

        # Failsafe mode (cannot escape without restarting drone_interface).
        self.print_debug("FAILSAFE")
        while True:
            # If we go into failsafe mode, continuously tell the flight
            # controller to land.
            self.vehicle.mode = dronekit.VehicleMode("LAND")
            time.sleep(0.1)

class CopterInterface:
    def __init__(self, address):
        self.vehicle = self.__connect_to_drone(address)
        self.controller = Controller(self.vehicle)
        self.sensor_reader = SensorReader(self.vehicle, self.controller)

        #TODO(comran): Implement interrupts better...
        self.interrupt = False

    def force_state(self, state):
        self.interrupt = True
        print("trying...")
        self.controller.set_state(state)
        print("state: " + self.controller.get_state())

    def stop(self):
        self.sensor_reader.stop()
        self.controller.stop()
        self.vehicle.close()

    def takeoff(self):
        self.controller.set_state("ARM")

        while True:
            if self.interrupt:
                self.interrupt = False
                return False

            if self.controller.get_state() == "ARMED":
                break
            time.sleep(0.1)

        print("Armed!")

        self.controller.set_state("TAKEOFF")

        while True:
            if self.interrupt:
                self.interrupt = False
                return False

            if self.controller.get_state() == "TAKEN OFF":
                break
            time.sleep(0.1)

        print("Taken off!")

        self.controller.set_state("VELOCITY CONTROL")

        return True

    def land(self):
        self.controller.set_state("LAND")
        print("Landing!")

    def goto(self, lat, lng, alt):
        while True:
            if not self.controller.get_state() == "VELOCITY CONTROL" \
                    or self.interrupt:
                return False

            sensors = self.sensor_reader.sensors.get()
            gps_lat = sensors["gps_lat"].get()
            gps_lng = sensors["gps_lng"].get()
            gps_rel_alt = sensors["gps_rel_alt"].get()

            if gps_lat is None or \
               gps_lng is None or \
               gps_rel_alt is None:
                time.sleep(0.1)
                continue

            TOLERANCE = 0.0001
            if abs(lat - gps_lat) < TOLERANCE and \
               abs(lng - gps_lng) < TOLERANCE and \
               abs(alt - gps_rel_alt) < 0.1:
               break

            vx = lat - gps_lat
            vy = lng - gps_lng
            vz = gps_rel_alt - alt

            vx *= 20000
            vy *= 20000

            self.controller.set_velocity(vx, vy, vz)

            print("vx: " + "%6s" % str("%0.1f" % vx) + \
                  " vy: " + "%6s" % str("%0.1f" % vy) + \
                  " vz: " + "%6s" % str("%0.1f" % vz) + \
                  " lat: " + "%11s" % str("%0.7f" % gps_lat) + \
                  " lng: " + "%11s" % str("%0.7f" % gps_lng) + \
                  " alt: " + "%6s" % str("%0.2f" % gps_rel_alt))

            time.sleep(0.1)

        return True

    def __connect_to_drone(self, address):
        # Initializes a Dronekit instance to interface with the flight
        # controller and returns this instance.

        print("Connecting to drone at " + address)

        vehicle = dronekit.connect(ip = address, \
                                   baud = 115200, \
                                   wait_ready = True)

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
