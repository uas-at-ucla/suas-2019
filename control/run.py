# Currently only uses simulated drone

import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
dname = os.path.dirname(os.path.realpath(__file__))
os.chdir(dname)

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../util')
sys.path.insert(0, 'flight_control')
sys.path.insert(0, 'commander')

import signal
import time

import process_manager
import copter_interface
import commander

TEST = True
drone_address = None # For when we have a real drone
processes = process_manager.ProcessManager()

def main():
    signal.signal(signal.SIGINT, kill_processes_and_exit)

    if TEST:
        init_lat = 38.1470000;
        init_lng = -76.4284722;
        drone_address = spawn_simulated_drone(init_lat, init_lng, 0.0, 0)

        processes.run_command("python ../ground/client/build.py")
        processes.spawn_process("python ../ground/run_ground.py")

    processes.spawn_process( \
            "python commander/drone_communications.py")        

    drone_commander = commander.Commander(drone_address)
    drone_commander.add_command(commander.TakeoffCommand())

    communications = drone_commander.get_communications_socket()
    communications.on('goto_command', on_goto_command)
    communications.on('start_mission', on_start_mission)
    communications.wait()

def on_start_mission():
    drone_commander.start_mission()
    drone_commander.clear_commands()

def on_goto_command(*args):
    position = args[0]
    drone_commander.add_command(commander.GotoCommand(position.lat, \
                                                      position.lng, \
                                                      position.alt))

def spawn_simulated_drone(lat, lng, alt, instance):
    processes.spawn_process("python " + \
            "flight_control/dronekit-sitl/dronekit_sitl/__init__.py " + \
            "copter " + \
            "--home " + str(lat) + "," \
                      + str(lng) + "," \
                      + str(alt) + ",0 " + \
            "--instance " + str(instance))

    # Wait to make sure the simulated drone is completely set up before
    # continuing.
    time.sleep(2.0)

    port = 5760 + 10 * instance
    return "tcp:127.0.0.1:" + str(port)

def kill_processes():
    processes.killall()

def kill_processes_and_exit(signal, frame):
    print "Exiting"
    kill_processes()
    exit(0)

if __name__ == "__main__":
    main()