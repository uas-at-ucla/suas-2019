import copter_interface

import os
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)

import pickle
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, dname + '/../flight_control')

import threading
from socketIO_client import SocketIO, BaseNamespace

import thread
import threading
import time

class Command:
    def __init__(self, data):
        #TODO: Support conversion from unicode.
        self.command_type = "unknown"
        for key, value in data.items():
            setattr(self, key, value)

class TakeoffCommand:
    def __init__(self):
        self.command_type = "takeoff"

class LandCommand:
    def __init__(self):
        self.command_type = "land"

class Commander:
    class CommunicationsNamespace(BaseNamespace):
        def on_connect():
            print('Commander connected to drone_communications!')

        def disconnect():
            print('Disconnected')

    def __init__(self, drone_address):
        self.mission_thread = None
        self.interrupt = False
        self.reset = True
        self.copter = None
        self.commands = list()
        self.commands_lock = threading.Lock()

        self.connect_to_drone_communications_thread = thread.start_new_thread( \
                self.connect_to_drone_communications, ())

        self.copter = copter_interface.CopterInterface(drone_address)
        self.copter.sensor_reader.set_communications_socket(self.communications)

    def execute_commands(self, *args):
        if self.copter is None:
            print("Rejected mission commands: Copter not ready.")
            return

        commands = args[0]
        self.add_command(TakeoffCommand())
        for command in commands:
            self.add_command(Command(command))
        self.add_command(LandCommand())

        self.interrupt = True
        self.copter.interrupt = True

        while not self.reset:
            print("Waiting for past mission to end.")
            time.sleep(0.5)
        self.interrupt = False
        self.copter.interrupt = False

        self.mission_thread = thread.start_new_thread( \
            self.start_mission, ())

    def set_state(self, *args):
        self.copter.force_state(args[0]['state'])

    def connect_to_drone_communications(self):
        print("Trying to connect to drone communications.")
        self.communications = SocketIO('0.0.0.0', 8085, \
                self.CommunicationsNamespace)
        print("Connected to drone communications.")

        self.communications.on('connect', lambda : None)
        self.communications.on('execute_commands', self.execute_commands)
        self.communications.on('set_state', self.set_state)
        self.communications.wait()

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
        self.reset = False
        commands = self.get_commands()

        for command in commands:
            if command.command_type == "takeoff":
                self.copter.takeoff()
            elif command.command_type == "goto":
                self.copter.goto(command.lat, command.lng, command.alt)
            elif command.command_type == "land":
                self.copter.land()
            else:
                print("UNKNOWN COMMAND!!!")
        self.reset = True

        self.clear_commands()
