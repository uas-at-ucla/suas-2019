# Currently only uses simulated drone

import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
dname = os.path.dirname(os.path.realpath(__file__))
os.chdir(dname)
os.chdir("control")

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../util')
sys.path.insert(0, 'flight_control')
sys.path.insert(0, 'commander')

import signal
import time
import sys
import thread
import argparse

import process_manager
import copter_interface
import commander

SIMULATE_DRONE = True

processes = process_manager.ProcessManager()

def spawn_simulated_drone(lat, lng, alt, instance, verbose):
    processes.spawn_process("python " + \
            "flight_control/dronekit-sitl/dronekit_sitl/__init__.py " + \
            "copter " + \
            "--home " + str(lat) + "," \
                      + str(lng) + "," \
                      + str(alt) + ",0 " + \
            "--instance " + str(instance), None, True, verbose)

    # Wait to make sure the simulated drone is completely set up before
    # continuing.
    time.sleep(2.0)

    port = 5760 + 10 * instance
    return "tcp:127.0.0.1:" + str(port)

def kill_processes():
    print("Exiting............................................................")
    processes.killall()
    exit(1)

def kill_processes_and_exit(signal, frame):
    # Signalled version.
    kill_processes()

def main():
    print("******************************************************************")
    print("*                                                                *")
    print("*                      UAS @ UCLA FlightDeck                     *")
    print("*                                                                *")
    print("******************************************************************")
    print("\n")
    signal.signal(signal.SIGINT, kill_processes_and_exit)

    parser = argparse.ArgumentParser()
    parser.add_argument('-v',               action="store_true")
    parser.add_argument('--simulator',      action="store_true")
    parser.add_argument('--ground',         action="store_true")
    parser.add_argument('--communications', action="store_true")
    parser.add_argument('--commander',      action="store_true")
    options = parser.parse_args()

    verbose                    = False
    drone_address              = "tcp:127.0.0.1:5760"
    run_all                    = True
    run_simulated_drone        = False
    run_ground                 = False
    run_commander              = False
    run_communications         = False

    if options.v:
        verbose                = True

    if options.simulator:
        run_all                = False
        run_simulated_drone    = True

    if options.ground:
        run_all                = False
        run_ground             = True

    if options.communications:
        run_all                = False
        run_communications     = True

    if options.commander:
        run_all                = False
        run_commander          = True

    if run_all:
        run_simulated_drone    = True
        run_ground             = True
        run_commander          = True
        run_communications     = True

    print("Starting the following:")
    if run_ground:
        print("  - Ground System")
    if run_simulated_drone:
        print("  - Drone Simulator")
    if run_communications:
        print("  - Drone Communications")
    if run_commander:
        print("  - Drone Commander")
    print("")


    # Now start all the components that the user asked us to start, and make
    # sure to kill everything if any errors occur.
    try:
        if run_simulated_drone:
            # Start the drone at the Webster Field that we will compete at.
            init_lat = 38.1470000;
            init_lng = -76.4284722;
            drone_address = spawn_simulated_drone(init_lat, init_lng, 0.0, 0, \
                    verbose)

        if run_ground:
            processes.spawn_process("python ../ground/client/build.py", None,
                    True, verbose)
            processes.spawn_process("python ../ground/run_ground.py", None,
                    True, verbose)

        if run_communications:
            processes.spawn_process( \
                    "python commander/drone_communications.py", None, True, \
                    verbose)

        if run_commander:
            global drone_commander
            drone_commander = commander.Commander(drone_address)
            communications = drone_commander.get_communications_socket()

    except Exception as e:
        print("ERROR: " + str(e))
        kill_processes()

    # Wait forever or until the user sends an interrupt signal.
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
