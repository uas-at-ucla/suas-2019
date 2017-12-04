import os
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)

import pickle
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, dname + '/../flight_control')

import threading
from socketIO_client import SocketIO, BaseNamespace

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
    class CommunicationsNamespace(BaseNamespace):
        def on_connect():
            print('Commander connected to drone_communications!')

        def disconnect():
            print('Disconnected')

    def __init__(self, drone_address):
        self.copter = copter_interface.CopterInterface(drone_address)

        self.commands = list()
        self.commands_lock = threading.Lock()

        self.communications = SocketIO('0.0.0.0', 8085, \
                self.CommunicationsNamespace)

        self.copter.sensor_reader.set_communications_socket(self.communications)

    def get_communications_socket(self):
        return self.communications

    def stop(self):
        self.copter.stop()
        self.save_mission_to_file()

    def save_mission_to_file(self):
        with open(dname+'/mission.pickle', 'wb') as f:
            pickle.dump(self.commands, f, pickle.HIGHEST_PROTOCOL)

    def add_command(self, command):
        with self.commands_lock:
            self.commands.append(command)

    def get_commands(self):
        with self.commands_lock:
            return self.commands[:]

    def clear_commands(self):
        self.commands = list()

    def start_mission(self):
        commands = self.get_commands()

        for command in commands:
            if command.command_type == "takeoff":
                self.copter.takeoff()
            elif command.command_type == "goto":
                self.copter.goto(command.lat, command.lng, command.alt)
            elif command.command_type == "unknown":
                print("UNKNOWN COMMAND!!!")
