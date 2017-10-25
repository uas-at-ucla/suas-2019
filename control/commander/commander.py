import os
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, dname + '../flight_control')

import threading

import copter_interface

class TakeoffCommand:
    def __init__(self):
        self.command_type = "takeoff"

class GotoCommand:
    def __init__(self, lat, lng, alt):
        self.command_type = "goto"
        self.lat = lat
        self.lng = lng
        self.alt = alt

class Commander:
    def __init__(self, drone_address):
        self.copter = copter_interface.CopterInterface(drone_address)

        self.commands = list()
        self.commands_lock = threading.Lock()

    def stop(self):
        self.copter.stop()

    def add_command(self, command):
        with self.commands_lock:
            self.commands.append(command)

    def get_commands(self):
        with self.commands_lock:
            return self.commands[:]

    def start_mission(self):
        commands = self.get_commands()

        for command in commands:
            if command.command_type == "takeoff":
                self.copter.takeoff()
            elif command.command_type == "goto":
                self.copter.goto(command.lat, command.lng, command.alt)
            elif command.command_type == "unknown":
                print("UNKNOWN COMMAND!!!")
