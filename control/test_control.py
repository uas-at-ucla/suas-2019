import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../util')
import time
import signal

import process_manager

def spawn_simulated_drone(proc_manager, lat, lng, alt, instance):
    proc_manager.spawn_process("python ./flight_control/simulate_copter.py " + \
            "copter " + \
            "--home " + str(lat) + "," + str(lng) + "," + str(alt) + ",0 " + \
            "--instance " + str(instance))

    port = 5760 + 10 * instance

    return "tcp:127.0.0.1:" + str(port)

def spawn_drone_interface(proc_manager, drone_address):
    proc_manager.spawn_process("python " + \
            "./flight_control/copter_interface.py " + \
            "--address " + drone_address)

def main():
    # Initialize a process manager to keep track of all spawned processes and
    # kill all these processes on exit.
    test_drone = process_manager.ProcessManager()
    signal.signal(signal.SIGINT, test_drone.terminate_all)

    drone_address = spawn_simulated_drone(test_drone, 0.0, 0.0, 0.0, 0)
    spawn_drone_interface(test_drone, drone_address)

    # Wait for program to terminate
    signal.pause()

if __name__ == "__main__":
    main()
