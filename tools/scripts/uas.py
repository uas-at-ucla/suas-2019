import os
import sys
import signal
import time
import argparse
import textwrap
import platform

os.chdir(os.path.dirname(os.path.realpath(__file__)))
os.chdir("../..")
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
DOCKER_RUN_SIM_SCRIPT   = "./tools/scripts/px4_simulator/start_sim.sh "
DOCKER_EXEC_SCRIPT      = "./tools/scripts/docker/exec.sh "
DOCKER_EXEC_KILL_SCRIPT = "./tools/scripts/docker/exec_kill.sh "

JENKINS_SERVER_START_SCRIPT = "./tools/scripts/jenkins_server/run_jenkins_server.sh "
LINT_CHECK_SCRIPT = "./tools/scripts/lint/check_format.sh"
LINT_FORMAT_SCRIPT = "./tools/scripts/lint/format.sh"

NUKE_SCRIPT = "./tools/scripts/nuke.sh"

# Command chains.
if "CONTINUOUS_INTEGRATION" in os.environ \
        and os.environ["CONTINUOUS_INTEGRATION"] == "true":

    # Limit verbosity in CI logs.
    BAZEL_BUILD = "bazel build --noshow_progress "
    BAZEL_TEST = "bazel test --noshow_progress "
else:
    BAZEL_BUILD = "bazel build "
    BAZEL_TEST = "bazel test "

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
    global received_signal
    if received_signal:
        print_update("ALREADY GOT SIGNAL RECEIVED ACTION! (be patient...)", \
                msg_type="FAILURE")
        return

    received_signal = True

    # Shutdown all the spawned processes and exit cleanly.
    print_update("performing signal received action...", msg_type="FAILURE")

    status = "Signal received (" + str(signal) + ") - killing all spawned " \
            "processes\n"

    global shutdown_functions
    shutdown_functions = set(shutdown_functions) # unique list
    for shutdown_function in shutdown_functions:
        status += shutdown_function()

    status += processes.killall()
    print_update(status, "FAILURE")
    sys.exit(0)


def kill_tmux_session(session):
    if processes.spawn_process_wait_for_code( \
            "tmux kill-session -t " + session, \
            show_output=False, allow_input=False) == 0:
        return "Killed tmux\n"
    return ""


def kill_tmux_session_uas_env():
    return kill_tmux_session("uas_env")


def kill_processes_in_uas_env_container():
    if processes.spawn_process_wait_for_code(DOCKER_EXEC_KILL_SCRIPT) == 0:
        return "Killed all spawned processes in docker image.\n"
    return ""


def kill_docker_container(name):
    command = "docker kill $(docker ps " \
              "--filter status=running " \
              "--format \"{{.ID}}\" " \
              "--filter name="+name+" " \
              "--latest)"
    if platform.system() == "Darwin":
        command = "eval $(docker-machine env uas-env); " + command
    return processes.spawn_process_wait_for_code(command, show_output=False, allow_input=False)


def kill_simulator():
    if kill_docker_container("uas-at-ucla_px4-simulator") == 0:
        return "Killed simulator (docker)\n"
    return ""


def kill_interop():
    if kill_docker_container("uas-at-ucla_interop-server") == 0:
        return "Killed interop server docker container\n"
    return ""


def run_and_die_if_error(command):
    if (processes.spawn_process_wait_for_code(command) != 0):
        processes.killall()
        sys.exit(1)


def run_cmd_exit_failure(cmd):
    if processes.spawn_process_wait_for_code(cmd, allow_input=False) > 0:
        status = "ERROR when running command: " + cmd + "\n" \
                "Killing all spawned processes\n"

        status += processes.killall()
        print_update(status, "FAILURE")

        sys.exit(1)


def kill_running_simulators():
    while kill_docker_container("uas-at-ucla_px4-simulator") == 0:
        print("killed sim")
        time.sleep(0.1)


def run_deploy(args):
    processes.spawn_process("python lib/scripts/deploy.py")
    processes.wait_for_complete()


def run_install(args=None):
    run_and_die_if_error("bash ./tools/scripts/install.sh")


def run_cleanup_docker(args):
    processes.spawn_process("./tools/scripts/docker/cleanup.sh")
    processes.wait_for_complete()
    print_update("Docker cleanup complete", "SUCCESS")


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


def run_build(args=None, show_complete=True):
    shutdown_functions.append(kill_processes_in_uas_env_container)

    print_update("Going to build the code...")

    print_update("Making sure all the necessary packages are installed.")
    run_install()

    # Start the UAS@UCLA software development docker image if it is not already
    # running.
    run_env(show_complete=False)

    # Execute the build commands in the running docker image.
    print_update("Downloading the dependencies...")

    print_update("Building src directory...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + BAZEL_BUILD + "//src/...")

    print_update("\n\nBuilding lib directory...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + BAZEL_BUILD + "//lib/...")

    print_update("\n\nBuilding src for raspi...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT \
            + BAZEL_BUILD + "--cpu=raspi //src/...")

    if show_complete:
        print_update("\n\nBuild successful :^) LONG LIVE SPINNY!", \
                msg_type="SUCCESS")


def run_unittest(args=None, show_complete=True):
    shutdown_functions.append(kill_processes_in_uas_env_container)

    print_update("Going to unittest the code...")

    print_update("Making sure all the necessary packages are installed.")
    run_install()

    # Start the UAS@UCLA software development docker image if it is not already
    # running.
    run_env(show_complete=False)

    # Execute the build commands in the running docker image.
    print_update("Testing src directory...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + BAZEL_TEST + "//src/...")

    print_update("\n\nTesting lib directory...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + BAZEL_TEST + "//lib/...")

    if show_complete:
        print_update("\n\nAll tests successful :^) LONG LIVE SPINNY!", \
                msg_type="SUCCESS")


def run_simulate(args):
    shutdown_functions.append(kill_processes_in_uas_env_container)
    shutdown_functions.append(kill_simulator)
    shutdown_functions.append(kill_tmux_session_uas_env)

    print_update("Building the code...")
    run_build(show_complete=False)

    print_update("Build complete! Starting simulator...")
    kill_running_simulators()

    print_update("Building UAS@UCLA software env docker...")

    # Build the image for our docker environment.
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + \
            "git clone " \
            "https://github.com/uas-at-ucla/Firmware.git " \
            "tools/cache/px4_simulator || true")

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

    run_cmd_exit_failure("tmux renamew -t uas_env controls")

    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + "rm /tmp/uasatucla_* || true")

    run_cmd_exit_failure("tmux split-window -h -p 50 -t uas_env")

    # Set up right side with 4 panes.
    run_cmd_exit_failure("tmux split-window -v -p 75 -t uas_env")
    run_cmd_exit_failure("tmux split-window -v -p 66 -t uas_env")
    run_cmd_exit_failure("tmux split-window -v -p 50 -t uas_env")

    # Set up left side with 3 panes
    run_cmd_exit_failure("tmux select-pane -t uas_env -L")
    run_cmd_exit_failure("tmux split-window -v -p 66 -t uas_env")
    run_cmd_exit_failure("tmux split-window -v -p 50 -t uas_env")
    run_cmd_exit_failure("tmux select-pane -t uas_env -U")
    run_cmd_exit_failure("tmux select-pane -t uas_env -U")

    # Run scripts on the left side.
    run_cmd_exit_failure("tmux send-keys \"" + DOCKER_RUN_SIM_SCRIPT + "\" C-m")

    run_cmd_exit_failure("tmux select-pane -t uas_env -D")

    run_cmd_exit_failure("tmux send-keys \"" + \
           "./tools/scripts/px4_simulator/exec_mavlink_router.sh\" C-m")

    run_cmd_exit_failure("tmux select-pane -t uas_env -D")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_EXEC_SCRIPT + " tail -F /tmp/drone_code.csv\" C-m")

    # Run scripts on the right side.
    run_cmd_exit_failure("tmux select-pane -t uas_env -R")
    run_cmd_exit_failure("tmux select-pane -t uas_env -U")
    run_cmd_exit_failure("tmux select-pane -t uas_env -U")
    run_cmd_exit_failure("tmux select-pane -t uas_env -U")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_EXEC_SCRIPT + \
            "bazel run //src/control/loops:flight_loop\" C-m")

    run_cmd_exit_failure("tmux select-pane -t uas_env -D")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_EXEC_SCRIPT + \
            "bazel run //src/control/io:io\" C-m")

    run_cmd_exit_failure("tmux select-pane -t uas_env -D")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_EXEC_SCRIPT + \
            "bazel run //src/control/ground_communicator:ground_communicator\" C-m")

    run_cmd_exit_failure("tmux select-pane -t uas_env -D")

    run_cmd_exit_failure("tmux send-keys \"" + \
            DOCKER_EXEC_SCRIPT + \
            "./tools/scripts/bazel_run.sh //lib/logger:log_writer\" C-m")

    print_update("\n\nSimulation running! \n" \
            "Run \"tmux a -t uas_env\" in another bash window to see everything working...", \
            msg_type="SUCCESS")

    while True:
        time.sleep(1)


def run_jenkins_server(args):
    print_update("Starting server...")

    # Create a Jenkins server and tunnel it to the uasatucla.org domain.
    processes.spawn_process(JENKINS_SERVER_START_SCRIPT)
    processes.spawn_process("while true;do " \
        "ssh -N -R 8082:localhost:8085 uas@uasatucla.org;sleep 1;done")

    print_update("Started Jenkins CI server!", msg_type="SUCCESS")
    processes.wait_for_complete()


def run_interop(args):
    shutdown_functions.append(kill_interop)

    print_update("Starting AUVSI interop server...")
    processes.spawn_process("./tools/scripts/ground/run_interop.sh")
    processes.wait_for_complete()


def run_ground(args):
    # Ground server and interface.
    if args.device is not None:
        processes.spawn_process("python ./src/ground/ground.py --device " \
                + args.device, None, True, args.verbose)
    else:
        processes.spawn_process("python ./src/ground/ground.py", None, True, \
                args.verbose)

    processes.wait_for_complete()


def run_env(args=None, show_complete=True):
    print_update("Starting UAS@UCLA development environment...")

    run_cmd_exit_failure(DOCKER_RUN_ENV_SCRIPT)

    if show_complete:
        print_update("UAS@UCLA development environment started " \
                "successfully!", msg_type="SUCCESS")


def run_nuke(args):
    run_cmd_exit_failure(NUKE_SCRIPT)

    print_update("Successfully nuked the UAS@UCLA development environment! " \
            ">:)", msg_type="SUCCESS")

def run_lint(args):
    print_update("Starting UAS@UCLA development environment...")
    run_cmd_exit_failure(DOCKER_RUN_ENV_SCRIPT)

    print_update("Running lint...")

    if args.format:
        print_update("Formatting the code...")
        run_cmd_exit_failure(LINT_FORMAT_SCRIPT)
        print_update("Format finished!", msg_type="SUCCESS")
    elif args.check:
        print_update("Checking lint...")
        run_cmd_exit_failure(LINT_CHECK_SCRIPT)
        print_update("Lint check passed!", msg_type="SUCCESS")
    else:
        print_update("NO LINTING OPTION SPECIFIED.", "FAILURE")


def run_help(args):
    print("./uas.sh")
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
    global received_signal
    received_signal = False
    signal.signal(signal.SIGINT, signal_received)

    global shutdown_functions
    shutdown_functions = []

    print(UAS_AT_UCLA_TEXT)

    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers()

    deploy_parser = subparsers.add_parser('deploy')
    deploy_parser.set_defaults(func=run_deploy)

    install_parser = subparsers.add_parser('install')
    install_parser.set_defaults(func=run_install)

    travis_parser = subparsers.add_parser('travis')
    travis_parser.set_defaults(func=run_travis)

    cleanup_docker_parser = subparsers.add_parser('cleanup_docker')
    cleanup_docker_parser.set_defaults(func=run_cleanup_docker)

    kill_dangling_parser = subparsers.add_parser('kill_dangling')
    kill_dangling_parser.set_defaults(func=run_kill_dangling)

    simulate_parser = subparsers.add_parser('simulate')
    simulate_parser.add_argument('--verbose', action='store_true')
    simulate_parser.set_defaults(func=run_simulate)

    interop_parser = subparsers.add_parser('interop')
    interop_parser.set_defaults(func=run_interop)

    ground_parser = subparsers.add_parser('ground')
    ground_parser.add_argument('--verbose', action='store_true')
    ground_parser.add_argument('--device', action='store', required=False)
    ground_parser.set_defaults(func=run_ground)

    build_parser = subparsers.add_parser('build')
    build_parser.set_defaults(func=run_build)

    unittest_parser = subparsers.add_parser('unittest')
    unittest_parser.set_defaults(func=run_unittest)

    run_env_parser = subparsers.add_parser('run_env')
    run_env_parser.set_defaults(func=run_env)

    jenkins_server_parser = subparsers.add_parser('jenkins_server')
    jenkins_server_parser.set_defaults(func=run_jenkins_server)

    lint_parser = subparsers.add_parser('lint')
    lint_parser.set_defaults(func=run_lint)
    lint_parser.add_argument('--format', action='store_true')
    lint_parser.add_argument('--check', action='store_true')

    nuke_parser = subparsers.add_parser('nuke')
    nuke_parser.set_defaults(func=run_nuke)

    help_parser = subparsers.add_parser('help')
    help_parser.set_defaults(func=run_help)

    args = parser.parse_args()
    args.func(args)
