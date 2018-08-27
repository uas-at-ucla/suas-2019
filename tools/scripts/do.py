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


# Script locations.
DOCKER_RUN_ENV_SCRIPT   = "./tools/scripts/docker/run_env.sh "
DOCKER_RUN_SIM_SCRIPT   = "./tools/scripts/docker/run_sim.sh "
DOCKER_EXEC_SCRIPT      = "./tools/scripts/docker/exec.sh "
DOCKER_EXEC_KILL_SCRIPT = "./tools/scripts/docker/exec_kill.sh "

# Command chains.
if "TRAVIS" in os.environ and os.environ["TRAVIS"] == "true":
    # Limit verbosity in Travis CI.
    BAZEL_BUILD = "bazel build --noshow_progress "
else:
    BAZEL_BUILD = "bazel build "

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

    if processes.spawn_process_wait_for_code(DOCKER_EXEC_KILL_SCRIPT) == 0:
        status += "Killed all spawned processes in docker image.\n"

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


def run_install(args=None):
    processes.spawn_process("bash ./tools/scripts/install.sh")
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
    print_update("Going to build the code...")

    print_update("Making sure all the necessary packages are installed.")
    run_install()

    # Start the UAS@UCLA software development docker image if it is not already
    # running.
    print_update("Bootstrapping UAS@UCLA environment...")
    run_cmd_exit_failure(DOCKER_RUN_ENV_SCRIPT)

    # Execute the build commands in the running docker image.
    print_update("Building src directory...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + BAZEL_BUILD + "//src/...")

    print_update("\n\nBuilding lib directory...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + BAZEL_BUILD + "//lib/...")

    print_update("\n\nBuilding src for raspi...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT \
            + BAZEL_BUILD + "--cpu=raspi //src/...")

    if show_complete:
        print_update("\n\nBuild complete :^) LONG LIVE SPINNY!", \
                msg_type="SUCCESS")


def run_simulate(args):
    print_update("Building the code...")
    run_build(show_complete=False)

    print_update("Build complete! Starting simulator...")
    kill_running_simulators()

    print_update("Building UAS@UCLA software env docker...")

    # Build the image for our docker environment.
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + \
            "git clone " \
            "https://github.com/PX4/Firmware.git " \
            "tools/docker/cache/px4_firmware || true")

    # Set up tmux panes.
    run_cmd_exit_failure("tmux start-server ")

    run_cmd_exit_failure("tmux kill-session " \
            "-t uas_env " \
            "> /dev/null || true")

    run_cmd_exit_failure("tmux " \
            "-2 " \
            "new-session " \
            "-d " \
            "-s " \
            "uas_env")

    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + "rm /tmp/uasatucla_* || true")

    run_cmd_exit_failure("tmux split-window -h -t uas_env")
    run_cmd_exit_failure("tmux split-window -v -t uas_env")
    run_cmd_exit_failure("tmux split-window -v -t uas_env")
    run_cmd_exit_failure("tmux select-pane -t uas_env -L")
    run_cmd_exit_failure("tmux split-window -v -t uas_env")
    run_cmd_exit_failure("tmux select-pane -t uas_env -U")
    run_cmd_exit_failure("tmux select-pane -t uas_env -U")
    run_cmd_exit_failure("tmux select-pane -t uas_env -U")

    # Start the PX4 simulator docker image.
    run_cmd_exit_failure("tmux send-keys \"exec " \
            + DOCKER_RUN_SIM_SCRIPT + "\" C-m")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-U")

    run_cmd_exit_failure("tmux renamew -t uas_env controls")

    run_cmd_exit_failure("tmux send-keys \"exec " + \
            "./tools/scripts/docker/run_mavproxy.sh\" C-m")

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
            DOCKER_EXEC_SCRIPT + \
            "bazel run //src/control/loops:flight_loop\" C-m")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-D")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_EXEC_SCRIPT + \
            "bazel run //src/control/io:io\" C-m")

    run_cmd_exit_failure("tmux select-pane " \
            "-t " \
            "uas_env " \
            "-D")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_EXEC_SCRIPT + \
            "bazel run //src/control/ground_communicator:ground_communicator\" C-m")

#   run_cmd_exit_failure("tmux send-keys \"" + \
#           DOCKER_EXEC_SCRIPT + \
#           "bazel run //src/control/ground_communicator:ground_communicator\" C-m")

    print_update("\n\nSimulation running! \n" \
            "Run \"tmux a -t uas_env\" in another bash window to see everything working...", \
            msg_type="SUCCESS")

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
