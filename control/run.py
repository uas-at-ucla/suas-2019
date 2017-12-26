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
import sys
import thread

import process_manager
import copter_interface
import commander

SIMULATE_DRONE = True

processes = process_manager.ProcessManager()

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
    print("Exiting............................................................")
    processes.killall()
    exit(1)

def kill_processes_and_exit(signal, frame):
    # Signalled version.
    kill_processes()

def main():
    signal.signal(signal.SIGINT, kill_processes_and_exit)

    run_simulated_drone = False
    run_ground          = False
    run_commander       = False
    run_communications  = False

    drone_address = "tcp:127.0.0.1:5760"

    # Read in the items that we want to run.
    run_list = "simulator,ground,commander,communications"
    if len(sys.argv) > 1:
        run_list = sys.argv[1]

    run_list = run_list.split(",")
    for to_run in run_list:
        if to_run == "simulator":
            run_simulated_drone = True
        elif to_run == "ground":
            run_ground = True
        elif to_run == "commander":
            run_commander = True
        elif to_run == "communications":
            run_communications = True
        else:
            print("ERROR: Cannot run item: \"" + to_run + "\"")
            sys.exit(1)

    # Now start all the components that the user asked us to start, and make
    # sure to kill everything if any errors occur.
    try:
        if run_simulated_drone:
            # Start the drone at the Webster Field that we will compete at.
            init_lat = 38.1470000;
            init_lng = -76.4284722;
            drone_address = spawn_simulated_drone(init_lat, init_lng, 0.0, 0)

        if run_ground:
            processes.run_command("python ../ground/client/build.py")
            processes.spawn_process("python ../ground/run_ground.py")

        if run_communications:
            processes.spawn_process( \
                    "python commander/drone_communications.py")

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
