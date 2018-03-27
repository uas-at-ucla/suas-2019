import os
import sys
import signal
import time

os.chdir(os.path.dirname(os.path.realpath(__file__)))
os.chdir("../../")
sys.dont_write_bytecode = True
sys.path.insert(0, 'lib')
import process_manager

BIN_FOLDER = './bazel-out/raspi-fastbuild/bin/'
DRONE_ADDR = 'pi@192.168.2.1'

processes = process_manager.ProcessManager()


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()
    sys.exit(0)


def upload_bin_to_drone(src_binary):
    src = BIN_FOLDER + src_binary
    dst = DRONE_ADDR + ':suas_2018_deploy/.'
    processes.spawn_process('rsync -avz --progress ' + src + ' ' + dst)

def run_cmd_on_drone(command):
    processes.spawn_process(
        'ssh ' + DRONE_ADDR + ' ' + command)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    #TODO(comran): Check if we can ssh into the drone before continuing the
    #              script.

    # Build the code that will be deployed.
    processes.spawn_process('bazel build --cpu=raspi //src/...')
    processes.wait_for_complete()
    processes.spawn_process('bazel build --cpu=raspi //aos/linux_code:core')
    processes.wait_for_complete()

    # Make a deploy directory, if it does not already exist.
    run_cmd_on_drone('mkdir -p ~/suas_2018_deploy')
    run_cmd_on_drone('killall core')
    run_cmd_on_drone('killall flight_loop')
    run_cmd_on_drone('killall io')
    run_cmd_on_drone('killall ground_communicator')
    run_cmd_on_drone('ipcrm --all')
    processes.wait_for_complete()

    # Upload the binaries to the drone.
    upload_bin_to_drone('aos/linux_code/core')
    upload_bin_to_drone('src/control/loops/flight_loop')
    upload_bin_to_drone('src/control/io/io')
    upload_bin_to_drone('src/control/ground_communicator/ground_communicator')
    processes.wait_for_complete()

    # Start binaries.
    run_cmd_on_drone('~/suas_2018_deploy/core&')
    run_cmd_on_drone('~/suas_2018_deploy/flight_loop&')
    run_cmd_on_drone('~/suas_2018_deploy/io&')
    run_cmd_on_drone('~/suas_2018_deploy/ground_communicator&')
    processes.wait_for_complete()
