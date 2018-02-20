# Currently only uses simulated drone

import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
dname = os.path.dirname(os.path.realpath(__file__))
os.chdir(dname)

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
sys.path.insert(0, 'flight_control')
sys.path.insert(0, 'commander')

import signal
import time
import sys
import thread
import argparse

import process_manager

SIMULATE_DRONE = True

processes = process_manager.ProcessManager()


def kill_processes():
    print(
        "Exiting............................................................")
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
    parser.add_argument('-v', action="store_true")
    parser.add_argument('--simulator3D', action="store_true")
    parser.add_argument('--simulatorGazeboHeadless', action="store_true")
    parser.add_argument('--simulator', action="store_true")
    parser.add_argument('--ground', action="store_true")
    parser.add_argument('--communications', action="store_true")
    parser.add_argument('--commander', action="store_true")
    options = parser.parse_args()

    verbose = False
    drone_address = "udp:127.0.0.1:14550"
    run_all = True
    run_simulated_drone = False
    run_3D_simulator = False
    run_headless_gazebo_simulator = False
    run_ground = False
    run_commander = False
    run_communications = False

    if options.v:
        verbose = True

    if options.simulator:
        run_all = False
        run_simulated_drone = True

    if options.simulatorGazeboHeadless:
        run_all = False
        run_simulated_drone = True
        run_headless_gazebo_simulator = True

    if options.simulator3D:
        run_all = False
        run_simulated_drone = True
        run_3D_simulator = True

    if options.ground:
        run_all = False
        run_ground = True

    if options.communications:
        run_all = False
        run_communications = True

    if options.commander:
        run_all = False
        run_commander = True

    if run_all:
        run_simulated_drone = True
        run_ground = True
        run_commander = True
        run_communications = True

    print("Starting the following:")
    if run_ground:
        print("  - Ground System")
    if run_simulated_drone:
        if run_3D_simulator:
            print("  - Drone Simulator (3D visualization))")
        else:
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
            processes.spawn_process( \
                    "socat pty,link=/tmp/virtualcom0,raw udp4-listen:14540", \
                    None, True, verbose)

            if run_headless_gazebo_simulator:
                processes.spawn_process(
                    "../../lib/scripts/bazel_run.sh " \
                            "@PX4_sitl//:gazebo",
                    None, True, verbose)

            elif run_3D_simulator:
                processes.spawn_process(
                    "../../lib/scripts/bazel_run.sh " \
                            "@PX4_sitl//:gazebo_visualize",
                    None, True, verbose)
            else:
                processes.spawn_process(
                    "../../lib/scripts/bazel_run.sh @PX4_sitl//:jmavsim",
                    None, True, verbose)

        if run_ground:
            ## Run this command to build final version
            # processes.spawn_process("python ../ground/client/build.py", None,
                                    # True, verbose)
            processes.spawn_process("python ../ground/run_ground.py", None,
                                    True, verbose)

        if run_communications:
            processes.spawn_process( \
                    "../../lib/scripts/bazel_run.sh //aos/linux_code:core", \
                    None, True, verbose)

            processes.spawn_process( \
                    "python python/commander/drone_communications.py", None, True, \
                    verbose)

        if run_commander:
            time.sleep(6)
            processes.spawn_process( \
                    "../../lib/scripts/bazel_run.sh //src/control/io:io", \
                    None, True, verbose)

            processes.spawn_process( \
                    "../../lib/scripts/bazel_run.sh " \
                    "//src/control/loops:flight_loop", None, True, verbose)

    except Exception as e:
        print("ERROR: " + str(e))
        kill_processes()

    # Wait forever or until the user sends an interrupt signal.
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
