import dronekit
import time
import websocket
import sys
import argparse
import math
import random
import threading
from datetime import datetime
from pymavlink import mavutil

sys.dont_write_bytecode = True

class SensorReader:
    def __init__(self, print_debug, flight_ctrl):
        self.main_logger = print_debug
        self.flight_ctrl = flight_ctrl

    def print_debug(self, message):
	msg = "(sensor_reader @ " + str(datetime.now()) + ") " + message
	self.main_logger(msg)

    def cut_from_string(self, raw, key):
        start = raw.find(key) + len(key)
        end = raw.find(",", start)
        if end is -1:
            end = len(raw)
        return raw[start:end]

    def get_telemetry(self, vehicle):
        return 'time:' + str(time.time()) + ',' \
             + 'status:"' + str(vehicle.system_status.state) + '",' \
             + 'state:"' + self.flight_ctrl.get_state() + '",' \
             + 'mode:"' + str(vehicle.mode.name) + '",' \
             + 'armed:"' + str(vehicle.armed) + '",' \
             + 'batt_volt:' + self.cut_from_string(str( \
                        vehicle.battery), "Battery:voltage=") + ',' \
             + 'last_heartbeat:' + str(vehicle.last_heartbeat) + ',' \
             + 'satellites:' + self.cut_from_string(str( \
                        vehicle.gps_0), "num_sat=") + ',' \
             + 'lat:' + self.cut_from_string(str( \
                        vehicle.location.global_frame), "lat=") + ',' \
             + 'lng:' + self.cut_from_string(str( \
                        vehicle.location.global_frame), "lon=") + ',' \
             + 'alt:' + self.cut_from_string(str( \
                        vehicle.location.global_frame), "alt=") + ',' \
             + 'rel_alt:' + self.cut_from_string(str( \
                        vehicle.location.global_relative_frame), \
                        "alt=") + ',' \
             + 'vel_x:' + str(vehicle.velocity[0]) + ',' \
             + 'vel_y:' + str(vehicle.velocity[1]) + ',' \
             + 'vel_z:' + str(vehicle.velocity[2]) + ',' \
             + 'pitch:' + self.cut_from_string(str( \
                        vehicle.attitude), "pitch=") + ',' \
             + 'roll:' + self.cut_from_string(str( \
                        vehicle.attitude), "roll=") + ',' \
             + 'yaw:' + self.cut_from_string(str( \
                        vehicle.attitude), "yaw=") + ',' \
             + 'heading:' + str(vehicle.heading) + ',' \
             + 'ground_speed:' + str(vehicle.groundspeed) + ',' \
             + 'air_speed:' + str(vehicle.airspeed)


    def read_telemetry(self, swim_ws, vehicle, uri):
        for i in range(0, 10):
            swim_ws.send('@command(node:"/swarm/0",lane:"register_drone")'
                               + '{uri:"' + uri + '"}')
            time.sleep(0.25)

        last_state = None
        while True:
            time.sleep(0.1)

            current_state = self.flight_ctrl.get_state()
            if last_state != current_state:
                swim_ws.send('@command(' \
                     + 'node:"' + uri + '",lane:"push_state") "' \
                     + self.flight_ctrl.get_state() + '"')
                last_state = current_state

            telemetry = self.get_telemetry(vehicle)

            self.print_debug("Flight control telemetry: " + telemetry)

            try:
                swim_ws.send('@command('
                           + 'node:"' + uri + '",lane:"push_telemetry"){' \
                           + telemetry + '}')
            except:
                break

sys.dont_write_bytecode = True

class Controller:
    def __init__(self, print_debug, uri):
	global print_debug_drone_interface
	self.main_logger = print_debug

        self.print_debug("NEW LOG")
        self.vel_lock = threading.Lock()
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.last_velocity = None
        self.state_lock = threading.Lock()
        self.state = "STANDBY"
        self.uri = uri

    def print_debug(self, message):
	msg = "(controller @ " + str(datetime.now()) + ") " + message
	self.main_logger(msg)

    def set_state(self, state):
        with self.state_lock:
            state = state.replace('"', '')
            if self.state == "FAILSAFE":
                self.print_debug("Failed to set state: In failsafe mode.")
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

    def check_velocity_control(self):
        # Make sure we are not sending stale commands.
        with self.vel_lock:
            if self.last_velocity is not None:
                diff = time.time() - self.last_velocity
                self.print_debug("Age of last velocity control from server: "
                        + str(diff))

                if diff > 0.25:
                    # Warn when velocity control starts to lag.
                    self.print_debug("Velocity control is lagging... " \
                            + "diff: " + str(diff))
                if diff > 0.50:
                    self.print_debug("VELOCITY CONTROL IS STALE, FS ENABLED")
                    return True
            return False

    def set_velocity(self, velocity_x, velocity_y, velocity_z):
	# Move vehicle in direction based on specified velocity vectors.
        # velocity_x is latitude
        # velocity_y is longitude
        # velocity_z is altitude (positive is down)

        with self.vel_lock:
            self.last_velocity = time.time()

            self.print_debug("Setting vel x: " + str(velocity_x) \
                    + " y: " + str(velocity_y) \
                    + " z: " + str(velocity_z))

            self.vel_x = velocity_x
            self.vel_y = velocity_y
            self.vel_z = velocity_z

    def send_velocity(self, vehicle):
        with self.vel_lock:
            velocity_x = self.vel_x
            velocity_y = self.vel_y
            velocity_z = self.vel_z

        cap = 1000
        velocity_x = max(min(cap, velocity_x), -1 * cap)
        velocity_y = max(min(cap, velocity_y), -1 * cap)
        cap = cap * 0.4
        velocity_z = max(min(cap, velocity_z), -1 * cap)

        msg = vehicle.message_factory \
                .set_position_target_local_ned_encode(
            0, # time_boot_ms (not used)
            0, 0, # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # 3D velocity
            0, 0, 0, # x, y, z acceleration (not supported yet)
            0, 0)    # yaw, yaw_rate (not supported yet)
        vehicle.send_mavlink(msg)

    def cut_from_string(self, raw, key):
        start = raw.find(key)
        if start == -1:
            return ""
        start = start + len(key)

        end = raw.find(",", start)
        if end == -1:
            end = raw.find("}", start)
        if end == -1:
            end = len(raw)
        return raw[start:end]

    def process(self, message, vehicle):
        body = str(message.split(')', 1)[-1])[1:-1]

        vel_x = self.cut_from_string(body, "vel_x:")
        vel_y = self.cut_from_string(body, "vel_y:")
        vel_z = self.cut_from_string(body, "vel_z:")

        if vel_x is not "" and vel_y is not "" and vel_z is not "":
            self.set_velocity(float(vel_x), float(vel_y), float(vel_z))
            return

        state = self.cut_from_string(body, "state:")
        if state is not "":
            self.set_state(state)

    def control_loop(self, vehicle):
        TARGET_TAKEOFF_ALT = 2.3

        last_loop = 0

        # Regular loop.
        while True:
            last_loop = self.phased_loop(10, last_loop)

            new_state = None
            current_state = self.get_state()

            # Log any change in state.
	    self.print_debug("Drone state: " + current_state)

            if current_state != "STANDBY" and self.check_velocity_control():
                self.set_state("FAILSAFE")
                current_state = "FAILSAFE"

            # Drone state machine.
            if current_state == "STANDBY":
                pass
            elif current_state == "ARM":
                self.set_state("ARM WAIT FOR INIT")
            elif current_state == "ARM WAIT FOR INIT":
                if vehicle.is_armable:
                    self.set_state("ARMING")
            elif current_state == "ARMING":
                vehicle.mode = dronekit.VehicleMode("GUIDED")
                vehicle.armed = True
                self.set_state("ARMED CHECK")
            elif current_state == "ARMED CHECK":
                if vehicle.armed:
                    self.set_state("ARMED")
            elif current_state == "ARMED":
                # Wait for other drones to also be armed.
                vehicle.mode = dronekit.VehicleMode("GUIDED")
                vehicle.armed = True
                pass
            elif current_state == "TAKEOFF":
                vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.armed = True
                self.set_state("TAKEOFF SEND COMMAND")
            elif current_state == "TAKEOFF SEND COMMAND":
                vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.armed = True
                vehicle.simple_takeoff(TARGET_TAKEOFF_ALT)
                self.set_state("TAKEOFF WAIT FOR TARGET ALTITUDE")
            elif current_state == "TAKEOFF WAIT FOR TARGET ALTITUDE":
                vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.armed = True
                if vehicle.location.global_relative_frame.alt \
                        >= TARGET_TAKEOFF_ALT * 0.85:
                    self.set_state("TAKEN OFF")
                else:
                    self.print_debug("Altitude: " +
                        str(vehicle.location.global_relative_frame.alt))
            elif current_state == "TAKEN OFF":
                # Wait for other drones to take off.
                vehicle.mode = dronekit.VehicleMode("GUIDED")
                self.armed = True
                pass
            elif current_state == "SWARM_CONTROL":
                vehicle.mode = dronekit.VehicleMode("GUIDED")
                vehicle.armed = True
                self.send_velocity(vehicle)
            elif current_state == "LAND":
                # Command vehicle to land.
                vehicle.mode = dronekit.VehicleMode("LAND")
                self.set_state("LANDING")
            elif current_state == "LANDING":
                # Wait until vehicle disarms after landing.
                if vehicle.armed:
                    self.print_debug("Altitude: " +
                        str(vehicle.location.global_relative_frame.alt))
                else:
                    self.set_state("LANDED")
            elif current_state == "LANDED":
                # Wait for all other drones to land.
                pass
            elif current_state == "FAILSAFE":
                # Exit control loop and go into failsafe mode.
                break
            else:
                self.print_debug("UNKNOWN CONTROLLER STATE: " + current_state)
                self.set_state("FAILSAFE")

        # Failsafe mode (cannot escape without restarting drone_interface).
        self.print_debug("FAILSAFE")
        while True:
            # If we go into failsafe mode, continuously tell the flight
            # controller to land.
            vehicle.mode = dronekit.VehicleMode("LAND")
            time.sleep(0.1)
