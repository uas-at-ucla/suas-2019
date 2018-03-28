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
DRONE_ADDR = 'pi@192.168.2.23'

processes = process_manager.ProcessManager()


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()
    sys.exit(0)


def upload_to_drone(src):
    dst = DRONE_ADDR + ':suas_2018_deploy/.'
    processes.spawn_process('sshpass -p "raspberry" rsync -avz --progress ' \
            + src + ' ' + dst)

def upload_bin_to_drone(src_binary):
    upload_to_drone(BIN_FOLDER + src_binary)

def run_cmd_on_drone(command):
    processes.spawn_process(
        'sshpass -p "raspberry" ssh -f ' + DRONE_ADDR \
                + ' ' + command)
    processes.wait_for_complete()

def print_title(text):
    number_of_hashes = 80 - len(text)
    hashes = ''
    for i in range(0, number_of_hashes):
        hashes += "#"

    print('\033[94m' + text + hashes + '\033[0m')

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    #TODO(comran): Check if we can ssh into the drone before continuing the
    #              script.

    # Build the code that will be deployed.
    print_title("Building code for raspi CPU... ")
    processes.spawn_process('bazel build --cpu=raspi //src/...')
    processes.wait_for_complete()
    processes.spawn_process('bazel build --cpu=raspi //aos/linux_code:core')
    processes.wait_for_complete()

    # Make a deploy directory, if it does not already exist.
    print_title("Setting up drone for deployment... ")
    run_cmd_on_drone('mkdir -p /home/pi/suas_2018_deploy')
    processes.wait_for_complete()

    # Upload the binaries to the drone.
    print_title("Uploading binaries to drone... ")
    upload_to_drone('lib/scripts/start_drone_code.sh')
    upload_bin_to_drone('aos/linux_code/core')
    upload_bin_to_drone('src/control/loops/flight_loop')
    upload_bin_to_drone('src/control/io/io')
    upload_bin_to_drone('src/control/ground_communicator/ground_communicator')
    processes.wait_for_complete()

    # Start binaries.
    print_title("Starting the new binaries on the drone... ")
    run_cmd_on_drone('/home/pi/suas_2018_deploy/start_drone_code.sh')
    processes.wait_for_complete()
