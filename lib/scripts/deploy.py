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
DRONE_ADDR = 'pi@192.168.1.20'

processes = process_manager.ProcessManager()

drone_code_uploads = list()


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()
    sys.exit(0)


def upload_to_drone(src, dst_path):
    dst = DRONE_ADDR + ':' + dst_path

    processes.spawn_process('sshpass -p "raspberry" rsync -avz --progress ' \
            + src + ' ' + dst)


def upload_bin_to_drone(src_binary, dst):
    upload_to_drone(BIN_FOLDER + src_binary, dst)


def run_cmd_on_drone(command):
    print('sshpass -p "raspberry" ssh -f ' + DRONE_ADDR \
                + ' "' + command + '"')
    processes.spawn_process(
        'sshpass -p "raspberry" ssh -f ' + DRONE_ADDR \
                + ' "' + command + '"')
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
    processes.spawn_process('bazel build --cpu=raspi //lib/logger/...')
    processes.wait_for_complete()

    # Make a deploy directory, if it does not already exist.
    print_title("Setting up drone for deployment... ")

    run_cmd_on_drone('mkdir -p /home/pi/logs')
    run_cmd_on_drone('mkdir -p /home/pi/pictures')
    run_cmd_on_drone('mkdir -p /home/pi/drone_code_deploy')
    processes.wait_for_complete()

    run_cmd_on_drone('mkdir -p /home/pi/logs/uas_at_ucla')
    run_cmd_on_drone('mkdir -p /home/pi/logs/mavproxy')
    run_cmd_on_drone('mkdir -p /home/pi/drone_code_deploy/scripts')
    run_cmd_on_drone('mkdir -p /home/pi/drone_code_deploy/executables')
    processes.wait_for_complete()
    run_cmd_on_drone('mkdir -p /home/pi/drone_code_deploy/scripts/serial_comms')
    processes.wait_for_complete()

    # Upload the binaries to the drone.
    print_title("Uploading binaries to drone... ")
    upload_to_drone('lib/scripts/raspi/start.sh', "/home/pi/.")
    upload_to_drone('lib/scripts/raspi/start_drone_code.sh',
                    "/home/pi/drone_code_deploy/scripts/.")
    upload_to_drone('lib/scripts/raspi/run_mavproxy.sh',
                    "/home/pi/drone_code_deploy/scripts/.")
    upload_to_drone('lib/scripts/download_photos.sh',
                    "/home/pi/drone_code_deploy/scripts/.")
    upload_to_drone('lib/scripts/take_photos_continuously.sh',
                    "/home/pi/drone_code_deploy/scripts/.")
    upload_to_drone('lib/scripts/tag_photos.sh',
                    "/home/pi/drone_code_deploy/scripts/.")
    upload_to_drone('lib/serial_comms/serial_comms.py',
                    "/home/pi/drone_code_deploy/scripts/serial_comms/.")
    upload_to_drone('lib/process_manager.py',
                    "/home/pi/drone_code_deploy/scripts/serial_comms/.")
    upload_to_drone('lib/serial_comms/serial_comms_message.proto',
                    "/home/pi/drone_code_deploy/scripts/serial_comms/.")
    upload_to_drone('lib/scripts/raspi/serial_comms_sender.sh',
                    "/home/pi/drone_code_deploy/scripts/serial_comms/.")

    upload_bin_to_drone('src/controls/loops/flight_loop',
                        "/home/pi/drone_code_deploy/executables/.")
    upload_bin_to_drone('src/controls/io/io',
                        "/home/pi/drone_code_deploy/executables/.")
    upload_bin_to_drone('src/controls/ground_communicator/ground_communicator',
                        "/home/pi/drone_code_deploy/executables/.")
    upload_bin_to_drone('lib/logger/log_writer',
                        "/home/pi/drone_code_deploy/executables/.")

    processes.wait_for_complete()

    run_cmd_on_drone('find \\"/home/pi/\\" ' + \
            '-not -path \\"/home/pi/\\" ' + \
            '-not -path \\"/home/pi/logs\\" ' + \
            '-not -path \\"/home/pi/logs/mavproxy\\" ' + \
            '-not -path \\"/home/pi/logs/uas_at_ucla\\" ' + \
            '-not -path \\"/home/pi/pictures\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/executables\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/serial_comms\\" ' + \
            '-not -path \\"/home/pi/\.*\\" ' + \
            '-not -path \\"/home/pi/start.sh\\" ' + \
            '-not -path \\"/home/pi/logs/mavproxy/*\\" ' + \
            '-not -path \\"/home/pi/logs/uas_at_ucla/*\\" ' + \
            '-not -path \\"/home/pi/pictures/*\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/start_drone_code.sh\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/run_mavproxy.sh\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/download_photos.sh\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/take_photos_continuously.sh\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/tag_photos.sh\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/serial_comms/serial_comms.py\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/serial_comms/serial_comms_sender.sh\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/serial_comms/process_manager.py\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/scripts/serial_comms/serial_comms_message.proto\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/executables/flight_loop\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/executables/io\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/executables/ground_communicator\\" ' + \
            '-not -path \\"/home/pi/drone_code_deploy/executables/log_writer\\" ' + \
            '-delete')
    processes.wait_for_complete()

    # Start binaries.
    print_title("Starting the new binaries on the drone... ")
    run_cmd_on_drone('/home/pi/drone_code_deploy/scripts/start_drone_code.sh')
    processes.wait_for_complete()
