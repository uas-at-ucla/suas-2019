import os
import sys
import signal
import time
import argparse
import textwrap

os.chdir(os.path.dirname(os.path.realpath(__file__)))
os.chdir("../../")
sys.dont_write_bytecode = True
sys.path.insert(0, 'lib')
import process_manager

processes = process_manager.ProcessManager()

UAS_AT_UCLA_TEXT = '\033[96m' + \
'################################################################################\n' + \
'####                                                                        ####\n' + \
'####         _   _   _    ____      ____    _   _  ____ _        _          ####\n' + \
'####        | | | | / \\  / ___|    / __ \\  | | | |/ ___| |      / \\         ####\n' + \
'####        | | | |/ _ \\ \\___ \\   / / _` | | | | | |   | |     / _ \\        ####\n' + \
'####        | |_| / ___ \\ ___) | | | (_| | | |_| | |___| |___ / ___ \\       ####\n' + \
'####         \___/_/   \\_\\____/   \\ \\__,_|  \\___/ \\____|_____/_/   \\_\\      ####\n' + \
'####                               \\____/                                   ####\n' + \
'####                                                                        ####\n' + \
'################################################################################\n' + \
'\033[0m'


def print_update(message, msg_type="STATUS"):
    SPLIT_SIZE = 65

    msg_split = message.splitlines()

    lines = list()
    for line in msg_split:
        lines.extend(textwrap.wrap(line, SPLIT_SIZE, break_long_words=False))

    print("\n")
    for i in range(0, len(lines) + 2):
        if i > 0 and i < len(lines) + 1:
            line = lines[i - 1]
        else:
            line = ""

        other_stuff = (5 + len(line) + 5)
        padding_left = (80 - other_stuff) / 2
        padding_right = 80 - padding_left - other_stuff
        print_line = ""

        if msg_type is "STATUS":
            print_line += "\033[94m"
        if msg_type is "STATUS_LIGHT":
            print_line += "\033[96m"
        elif msg_type is "SUCCESS":
            print_line += "\033[92m"
        elif msg_type is "FAILURE":
            print_line += "\033[91m"

        print_line += "#### "
        print_line += " " * padding_left
        print_line += line
        print_line += " " * padding_right
        print_line += " ####\033[0m"

        print(print_line)


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.

    print_update("performing signal received action...", msg_type="FAILURE")

    status = "Signal received (" + str(signal) + ") - killing all spawned " \
            "processes\n"

    if processes.spawn_process_wait_for_code("docker kill $(docker ps " \
            "--filter status=running " \
            "--format \"{{.ID}}\" " \
            "--filter name=uas_sim " \
            "--latest)", show_output=False, allow_input=False) == 0:
        status += "Killed simulator (docker)\n"

    if processes.spawn_process_wait_for_code( \
            "tmux kill-session -t uas_env", \
            show_output=False, allow_input=False) == 0:
        status += "Killed tmux\n"

    DOCKER_PREFIX_CMD = "docker exec -t $(docker ps " \
            "--filter status=running " \
            "--filter name=uas_env " \
            "--format \"{{.ID}}\" " \
            "--latest) "

    if processes.spawn_process_wait_for_code( \
            DOCKER_PREFIX_CMD + \
            "bash lib/scripts/docker_exec_kill.sh", \
            show_output=False, allow_input=False) == 0:
        status += "Killed screen sessions within uas_env\n"

    status += processes.killall()
    print_update(status, "FAILURE")
    sys.exit(0)


def run_and_die_if_error(command):
    if (processes.spawn_process_wait_for_code(command) != 0):
        processes.killall()
        sys.exit(1)


def run_deploy(args):
    processes.spawn_process("python lib/scripts/deploy.py")
    processes.wait_for_complete()


def run_install(args):
    if os.getuid() != 0:
        print("ERROR: Must be sudo to run install script.")
        sys.exit(1)

    processes.spawn_process("sudo bash tools/installation/install.sh")
    processes.wait_for_complete()


def run_kill_dangling(args):
    processes.spawn_process("killall java")
    processes.spawn_process("killall px4")
    processes.spawn_process("killall io")
    processes.spawn_process("killall flight_loop")
    processes.spawn_process("killall flight_loop_lib_test")
    processes.spawn_process("killall ground_communicator")
    processes.spawn_process("killall node")
    processes.wait_for_complete()
    processes.spawn_process("killall python")
    processes.wait_for_complete()


def run_travis(args):
    run_and_die_if_error("bazel build //src/...")
    run_and_die_if_error("bazel build --cpu=raspi //src/...")
    run_and_die_if_error("bazel build @PX4_sitl//:jmavsim")
    run_and_die_if_error("bazel test //src/...")
    run_and_die_if_error("bazel test //lib/...")
    run_and_die_if_error(
        "./bazel-out/k8-fastbuild/bin/src/control/loops/flight_loop_lib_test")

def run_cmd_exit_failure(cmd):
    if processes.spawn_process_wait_for_code(cmd, allow_input=False) > 0:
        status = "ERROR when running command: " + cmd + "\n" \
                "Killing all spawned processes\n"

        status += processes.killall()
        print_update(status, "FAILURE")

        sys.exit(1)

def is_uasatucla_dev_env_running():
    return processes.spawn_process_wait_for_code( \
            "if [ -z $(docker ps --filter status=running " \
                                "--format \"{{.ID}}\" --latest " \
                                "--filter name=uas_env) ]; " \
            "then exit 1; " \
            "else exit 0; fi") == 0

def kill_running_simulators():
    while processes.spawn_process_wait_for_code("docker kill $(docker ps " \
            "--filter status=running " \
            "--filter name=uas_sim " \
            "--format \"{{.ID}}\" --latest)", \
            show_output=False, \
            allow_input=False) == 0:
        print("killed sim")
        time.sleep(0.1)


def run_build(args=None, show_complete=True):
    # Start the UAS@UCLA software development docker image if it is not already
    # running.
    print_update("Going to build the code...")

    if not is_uasatucla_dev_env_running():
        print_update("Building UAS@UCLA software env docker...")
        # Build the image for our docker environment.
        # Add --no-cache to avoid using cache.
        run_cmd_exit_failure("docker build -t uas-at-ucla_software " \
                "tools/docker")

        run_cmd_exit_failure("docker network create -d bridge uas_bridge " \
                " > /dev/null || true")

        run_cmd_exit_failure("docker run " \
                "-it " \
                "-d " \
                "--rm " \
                "--net uas_bridge " \
                "-v $(pwd):/home/uas/code_env/ " \
                "-v $(pwd)/tools/docker/cache/bazel:" \
                    "/home/uas/.cache/bazel/_bazel_uas " \
                "--name uas_env " \
                "--env=LOCAL_USER_ID=\"$(id -u)\" " \
                "uas-at-ucla_software " \
                "\"bazel | tail -f /dev/null\"")

    while not is_uasatucla_dev_env_running():
        time.sleep(0.25)

    # Execute the build commands in the running docker image.
    DOCKER_PREFIX_CMD = "docker exec -t $(docker ps " \
            "--filter status=running " \
            "--filter name=uas_env " \
            "--format \"{{.ID}}\" " \
            "--latest) "

    print_update("Building src directory...")
    run_cmd_exit_failure(DOCKER_PREFIX_CMD + "bazel build //src/...")

    print_update("\n\nBuilding lib directory...")
    run_cmd_exit_failure(DOCKER_PREFIX_CMD + "bazel build //lib/...")

    print_update("\n\nBuilding shm core...")
    run_cmd_exit_failure(DOCKER_PREFIX_CMD + \
            "bazel build //aos/linux_code:core")

    print_update("\n\nBuilding src for raspi...")
    run_cmd_exit_failure(DOCKER_PREFIX_CMD + \
            "bazel build --cpu=raspi //src/...")

    print_update("\n\nBuilding shm core for raspi...")
    run_cmd_exit_failure(DOCKER_PREFIX_CMD + \
            "bazel build --cpu=raspi //aos/linux_code:core")

    if show_complete:
        print_update("\n\nBuild complete :^) LONG LIVE SPINNY!", \
                msg_type="SUCCESS")


def run_simulate(args):
    DOCKER_PREFIX_CMD = "./lib/scripts/docker_exec.sh " \
            "$(docker ps " \
            "--filter status=running " \
            "--filter name=uas_env " \
            "--format \"{{.ID}}\" " \
            "--latest) "

    print_update("Building the code...")
    run_build(show_complete=False)

    print_update("Build complete! Starting simulator...")
    kill_running_simulators()

    print_update("Building UAS@UCLA software env docker...")

    # Build the image for our docker environment.
    run_cmd_exit_failure(DOCKER_PREFIX_CMD + \
            "git clone " \
            "https://github.com/PX4/Firmware.git " \
            "tools/docker/cache/px4_firmware || true")

    # Set up tmux panes.
    run_cmd_exit_failure("tmux kill-session " \
            "-t uas_env " \
            "> /dev/null || true")

    run_cmd_exit_failure("tmux " \
            "-2 " \
            "new-session " \
            "-d " \
            "-s " \
            "uas_env")

    run_cmd_exit_failure("tmux split-window " \
            "-h " \
            "-t " \
            "uas_env")

    run_cmd_exit_failure("tmux split-window " \
            "-v " \
            "-t " \
            "uas_env")

    run_cmd_exit_failure("tmux split-window " \
            "-v " \
            "-t " \
            "uas_env")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-L")

    run_cmd_exit_failure("tmux split-window " \
            "-v " \
            "-t " \
            "uas_env")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-U")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-U")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-U")

    # Start the PX4 simulator docker image.
    run_cmd_exit_failure("tmux send-keys \"" \
            "docker run --rm -t " \
            "--env=LOCAL_USER_ID=\\\"$(id -u)\\\" " \
            "-v $(pwd)/tools/docker/cache/px4_firmware:/src/firmware/:rw " \
            "-v /tmp/.X11-unix:/tmp/.X11-unix:ro " \
            "-e DISPLAY=:0 "\
            "--name uas_sim " \
            "--net uas_bridge " \
            "-p 14556:14556/udp " \
            "px4io/px4-dev-ros:2017-10-23 " \
            "/bin/sh -c \\\"cd /src/firmware;" \
            "HEADLESS=1 make posix_sitl_default gazebo\\\"\" C-m")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-U")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_PREFIX_CMD + \
            "/home/uas/.local/bin/mavproxy.py " \
            "--nowait " \
            "--show-errors " \
            "--master udpout:172.19.0.3:14557 " \
            "--out udp:0.0.0.0:8084 --non-interactive\" C-m")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-R")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-U")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-U")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_PREFIX_CMD + \
            "bazel run //src/control/loops:flight_loop\" C-m")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-D")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_PREFIX_CMD + \
            "bazel run //src/control/io:io\" C-m")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-D")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_PREFIX_CMD + \
            "bazel run //src/control/ground_communicator:ground_communicator\" C-m")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_PREFIX_CMD + \
            "bazel run //lib/logger:log_writer\" C-m")

    while True:
        time.sleep(1)


def run_ground(args):
    # Ground server and interface.
    if args.device is not None:
        processes.spawn_process("python ./src/ground/ground.py --device " \
                + args.device, None, True, args.verbose)
    else:
        processes.spawn_process("python ./src/ground/ground.py", None, True, \
                args.verbose)

    processes.wait_for_complete()


def run_help(args):
    print("./do.sh")
    print("      > help")
    print("      > build")
    print("      > test")
    print("        controls")
    print("               > build")
    print("               > test")
    print("               > simulate")
    print("")
    print("        ground")
    print("               > run")
    print("               > test")
    print("")
    print("        vision")
    print("               > test")
    print("               > train")


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    print(UAS_AT_UCLA_TEXT)

    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers(help='sub-command help')

    deploy_parser = subparsers.add_parser('deploy', help='deploy help')
    deploy_parser.set_defaults(func=run_deploy)

    install_parser = subparsers.add_parser('install', help='install help')
    install_parser.set_defaults(func=run_install)

    travis_parser = subparsers.add_parser('travis', help='travis help')
    travis_parser.set_defaults(func=run_travis)

    kill_dangling_parser = subparsers.add_parser(
        'kill_dangling', help='kill_dangling help')
    kill_dangling_parser.set_defaults(func=run_kill_dangling)

    simulate_parser = subparsers.add_parser('simulate', help='simulate help')
    simulate_parser.add_argument(
        '--verbose', action='store_true', help='verbose help')
    simulate_parser.set_defaults(func=run_simulate)

    ground_parser = subparsers.add_parser('ground', help='ground help')
    ground_parser.add_argument(
        '--verbose', action='store_true', help='verbose help')
    ground_parser.add_argument(
        '--device', action='store', help='device help', required=False)
    ground_parser.set_defaults(func=run_ground)

    build_parser = subparsers.add_parser('build', help='build help')
    build_parser.set_defaults(func=run_build)

    help_parser = subparsers.add_parser('help')
    help_parser.set_defaults(func=run_help)

    args = parser.parse_args()
    args.func(args)
