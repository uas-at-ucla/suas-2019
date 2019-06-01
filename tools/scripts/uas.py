import os
import sys
import signal
import time
import math
import argparse
import textwrap
import platform
import subprocess
import multiprocessing

os.chdir(os.path.dirname(os.path.realpath(__file__)))
os.chdir("../..")
sys.dont_write_bytecode = True
sys.path.insert(0, 'lib')
import process_manager

processes = process_manager.ProcessManager()

UAS_AT_UCLA_TEXT = '\033[96m' + \
'################################################################################\n' + \
'####                                                                        ####\n' + \
'####                                UAS@UCLA                                ####\n' + \
'####                                                                        ####\n' + \
'################################################################################\n' + \
'\033[0m'


# Script locations.
GROUND_DOCKER_EXEC_SCRIPT = "./tools/scripts/ground/exec.sh "

DOCKER_RUN_ENV_SCRIPT   = "./tools/scripts/controls/run_env.sh "
DOCKER_RUN_SIM_SCRIPT   = "./tools/scripts/px4_simulator/start_sim.sh "
DOCKER_EXEC_SCRIPT      = "./tools/scripts/controls/exec.sh "
DOCKER_EXEC_KILL_SCRIPT = "./tools/scripts/controls/exec_kill.sh "

VISION_DOCKER_BUILD_SCRIPT  = "./tools/scripts/docker/vision/docker_build.sh "
VISION_DOCKER_RUN_SCRIPT    = "./tools/scripts/docker/vision/docker_run.sh "

CONTROLS_DEPLOY_SCRIPT = "./tools/scripts/controls/deploy.sh "
CONTROLS_TEST_RRT_AVOIDANCE_SCRIPT = "./bazel-out/k8-fastbuild/bin/lib/rrt_avoidance/rrt_avoidance_test --plot"
CONTROLS_MAVROS_SCRIPT = "./tools/scripts/controls/run_mavros.sh"

JENKINS_SERVER_START_SCRIPT = "./tools/scripts/jenkins_server/run_jenkins_server.sh "
JENKINS_SLAVE_START_SCRIPT = "./tools/scripts/jenkins_slave/start_jenkins_slave.sh"
LINT_CHECK_SCRIPT = "./tools/scripts/lint/check_format.sh"
LINT_FORMAT_SCRIPT = "./tools/scripts/lint/format.sh"

NUKE_SCRIPT = "./tools/scripts/nuke.sh"

# IP_SCRIPT = "./tools/scripts/controls/get_ip.sh"

# DOCKER_IP = subprocess.check_output(IP_SCRIPT, shell=True)

# Command chains.
if "CONTINUOUS_INTEGRATION" in os.environ \
        and os.environ["CONTINUOUS_INTEGRATION"] == "true":
    print("CI ENABLED!")

    # Limit verbosity in CI logs.
    meminfo = dict((i.split()[0].rstrip(':'),int(i.split()[1])) for i in open('/proc/meminfo').readlines())
    mem_kib = meminfo['MemAvailable']

    CI_BUILD_RAM = mem_kib / 1024 # MB
    CI_BUILD_CPUS = multiprocessing.cpu_count()
    CI_BUILD_IO = 1.0

    print("RAM available: " + str(CI_BUILD_RAM) + "MB")
    print("CPUs available: " + str(CI_BUILD_CPUS))

    CI_BUILD_LOCAL_RESOURCES = ""
    #CI_BUILD_LOCAL_RESOURCES = "--local_resources " + str(CI_BUILD_RAM) + "," \
    #        + str(CI_BUILD_CPUS) + "," \
    #        + str(CI_BUILD_IO)

    BAZEL_BUILD = "bazel build --noshow_progress " + CI_BUILD_LOCAL_RESOURCES + " "
    BAZEL_TEST = "bazel test --noshow_progress " + CI_BUILD_LOCAL_RESOURCES + " "
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


def kill_processes_in_uas_ground_container():
    if processes.spawn_process_wait_for_code("./tools/scripts/ground/exec_kill.sh") == 0:
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


def kill_controls():
    if kill_docker_container("uas-at-ucla_controls") == 0:
        return "Killed controls docker container\n"
    return ""


def kill_controls_rqt():
    if kill_docker_container("uas-at-ucla_ros-rqt") == 0:
        return "Killed controls ros-rqt docker container\n"
    return ""


def kill_ground():
    if kill_docker_container("uas-at-ucla_ground") == 0:
        return "Killed ground docker container\n"
    return ""


def kill_jenkins_client():
    if kill_docker_container("uas-at-ucla_jenkins-slave") == 0:
        return "Killed jenkins slave docker container\n"
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


def tmux_cmd(cmd):
    run_cmd_exit_failure("tmux send-keys \"unset HISTFILE;" + cmd + "\" C-m")


def tmux_move_pane(direction):
    if direction == "down":
        run_cmd_exit_failure("tmux select-pane -t uas_env -D")
    elif direction == "up":
        run_cmd_exit_failure("tmux select-pane -t uas_env -U")
    elif direction == "right":
        run_cmd_exit_failure("tmux select-pane -t uas_env -R")
    elif direction == "left":
        run_cmd_exit_failure("tmux select-pane -t uas_env -L")
    else:
        print("Unknown direction: " + direction)
        signal_received(2, 0)


def tmux_select_window(index):
    run_cmd_exit_failure("tmux select-window -t uas_env:" + str(index))


def tmux_split(direction, times):
    direction_flag = None
    direction_move = None

    if direction == "horizontal":
        direction_flag = "-h"
        direction_move = "left"
    elif direction == "vertical":
        direction_flag = "-v"
        direction_move = "up"
    else:
        print("Unknown direction: " + direction)
        sys.exit(1)

    # Split the pane.
    for i in range(0, times - 1):
        amount = int(math.floor((1 - 1.0 / (times - i)) * 100))
        run_cmd_exit_failure("tmux split-window " \
            + direction_flag \
            + " -p " + str(amount) \
            + " -t uas_env")

    # Move back to the top left.
    for i in range(0, times - 1):
        tmux_move_pane(direction_move)



def tmux_rename_pane(name):
    run_cmd_exit_failure("tmux rename-window -t uas_env \"" + name + "\"")


def tmux_new_window(name):
    run_cmd_exit_failure("tmux new-window -t uas_env -n " + name)


def create_link(src, dst):
    run_cmd_exit_failure("ln -sf " + src + " " + dst)


def kill_running_simulators():
    while kill_docker_container("uas-at-ucla_px4-simulator") == 0:
        print("killed sim")
        time.sleep(0.1)


def run_vision_build(args):
    print_update("Building vision docker image...")
    run_cmd_exit_failure(VISION_DOCKER_BUILD_SCRIPT)
    print_update(
        "\n\nVision docker image successfully built!", msg_type="SUCCESS")


def run_vision(args):
    print_update("Running vision docker image...")
    run_cmd_exit_failure(VISION_DOCKER_RUN_SCRIPT + " ".join(args.vision_args))


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
        "./bazel-out/k8-fastbuild/bin/src/controls/loops/flight_loop_lib_test")


def run_controls_build(args=None, show_complete=True, raspi=True):
    shutdown_functions.append(kill_processes_in_uas_env_container)

    # Create link to executables folder.
    directory = os.path.dirname(os.path.realpath(__file__))

    #create_link("../tools/cache/bazel/execroot/com_uclauas/external", "external/downloaded")
    #create_link("tools/cache/bazel/execroot/com_uclauas/bazel-out", "output")

    print_update("Going to build the code...")

    run_controls_docker_start(None, show_complete=False)

    print_update("Building src/lib directory for AMD64...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + BAZEL_BUILD + "//src/... //lib/...")

    if raspi:
        print_update("\n\nBuilding src for raspi...")
        run_cmd_exit_failure(DOCKER_EXEC_SCRIPT \
                + BAZEL_BUILD + "--cpu=raspi //src/...")

    if show_complete:
        print_update("\n\nBuild successful :^) LONG LIVE SPINNY!", \
                msg_type="SUCCESS")


def run_controls_deploy(args=None):
    run_controls_build(show_complete=False)

    print_update("Deploying to raspi...")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + CONTROLS_DEPLOY_SCRIPT \
            + "src/controls/io/io io")
    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + CONTROLS_DEPLOY_SCRIPT \
            + "src/controls/flight_loop/flight_loop flight_loop")

    run_cmd_exit_failure(DOCKER_EXEC_SCRIPT + CONTROLS_DEPLOY_SCRIPT \
            + "src/controls/ground_communicator/ground_communicator ground_communicator")

def run_controls_rqt(args=None):
    shutdown_functions.append(kill_controls_rqt)

    print_update("Starting ROS rqt...")

    run_cmd_exit_failure("./tools/scripts/controls/run_rqt.sh")


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


def run_controls_sitl(args):
    processes.spawn_process("./uas controls simulate", show_output=False)
    return_code = processes.spawn_process_wait_for_code(DOCKER_EXEC_SCRIPT \
        + "timeout 3600s rostopic echo -p -n 100 " \
        + "/mavros/global_position/global/altitude")

    if return_code == 0:
        print_update("Success!", msg_type="SUCCESS")
        processes.killall()
        sys.exit(0)
    else:
        print_update("Simulator timed out :(", msg_type="FAILURE")
        processes.killall()
        sys.exit(1)


def run_controls_simulate(args):
    shutdown_functions.append(kill_processes_in_uas_env_container)
    shutdown_functions.append(kill_simulator)
    shutdown_functions.append(kill_tmux_session_uas_env)
    shutdown_functions.append(kill_ground)

    run_controls_docker_start(None, show_complete=False)

    # Make sure the simulator is not already running.
    if processes.spawn_process_wait_for_code( \
            "tmux has-session -t uas_env > /dev/null 2>&1") == 0:
        print_update("UAS simulation is already running!", \
                msg_type="FAILURE")
        sys.exit(1)

    run_cmd_exit_failure("tmux kill-session " \
            "-t uas_env " \
            "> /dev/null 2>&1 || true")

    kill_running_simulators()

    # Build the docker image for our PX4 simulator environment.
    print_update("Building UAS@UCLA software env docker...")

    # Set up tmux panes.
    run_cmd_exit_failure("tmux start-server")

    run_cmd_exit_failure("tmux " \
            "-2 " \
            "new-session " \
            "-d " \
            "-s " \
            "uas_env")

    tmux_rename_pane("PX4")

    sim_command = DOCKER_RUN_SIM_SCRIPT
    mavlink_router_command = "./tools/scripts/px4_simulator/exec_mavlink_router.sh"
    io_command = "bazel run //src/controls/io:io 0.0.0.0"
    if args.lite:
        sim_command = "echo 'Not running simulator'"
        mavlink_router_command = "echo 'Not running mavlink router'"
        io_command = "bazel run //src/controls/io_sim:io_sim"

    tmux_split("horizontal", 2)
    tmux_cmd(sim_command)

    tmux_move_pane("right")
    tmux_cmd(mavlink_router_command)

    tmux_new_window("MAVROS")
    tmux_cmd(DOCKER_EXEC_SCRIPT + CONTROLS_MAVROS_SCRIPT)

    tmux_new_window("ROS")
    tmux_split("horizontal", 2)
    tmux_cmd(DOCKER_EXEC_SCRIPT + "bazel run //src/controls/flight_loop:flight_loop")
    tmux_move_pane("right")
    tmux_cmd(DOCKER_EXEC_SCRIPT + io_command)
    tmux_split("vertical", 2)
    tmux_move_pane("down")
    tmux_cmd(DOCKER_EXEC_SCRIPT + "bazel run //src/controls/ground_communicator:ground_communicator")

    tmux_new_window("GROUND")
    tmux_split("horizontal", 2)
    tmux_cmd(DOCKER_EXEC_SCRIPT + "bazel run //src/controls/ground_controls:ground_controls 192.168.3.10")
    tmux_move_pane("right")
    tmux_cmd("./uas ground run server")

    tmux_select_window(2)

    print_update("\n\nSimulation running! \n" \
            "Run \"tmux a -t uas_env\" in another bash window to see everything working. Use `Ctrl-B n` to cycle between windows.", \
            msg_type="SUCCESS")

    while True:
        if processes.spawn_process_wait_for_code( \
            "tmux has-session -t uas_env > /dev/null 2>&1") == 1:
            print_update("Tmux session closed, killing simulator...", "FAILURE")
            # Session exited.
            signal_received(2, 0)

        time.sleep(1)


def run_jenkins_server(args):
    print_update("Starting server...")

    # Create a Jenkins server and tunnel it to the uasatucla.org domain.
    processes.spawn_process(JENKINS_SERVER_START_SCRIPT)

    print_update("Started Jenkins CI server!", msg_type="SUCCESS")
    processes.wait_for_complete()


def run_jenkins_client(args):
    shutdown_functions.append(kill_jenkins_client)

    print_update("Starting client...")

    # Create a Jenkins server and tunnel it to the uasatucla.org domain.
    processes.spawn_process(JENKINS_SLAVE_START_SCRIPT + " " + args.auth, allow_input=False)

    print_update("Started Jenkins CI server!", msg_type="SUCCESS")
    processes.wait_for_complete()


def run_interop(args):
    shutdown_functions.append(kill_interop)

    print_update("Starting AUVSI interop server...")
    processes.spawn_process("./tools/scripts/ground/run_interop.sh")
    processes.wait_for_complete()

def run_ground_build(args):
    run_ground("build", args)

def run_ground_run(args):
    run_ground("run --web", args)

def run_ground(arg1, args):
    shutdown_functions.append(kill_ground)

    print_update("Starting the Ground Station Docker container...")
    run_cmd_exit_failure("./tools/scripts/ground/run_env.sh")
    print_update("Running the Ground Station...")

    # Run ground.py and pass command line arguments
    run_cmd_exit_failure(GROUND_DOCKER_EXEC_SCRIPT + \
           "'ssh -o StrictHostKeyChecking=no git@github.com; sudo python3 ./src/ground/ground.py " + arg1 + " " + " ".join(args.ground_args) + "'")

    kill_ground()

def run_ground_docker(args):
    shutdown_functions.append(kill_ground)

    print_update("Starting the Ground Station Docker container...")
    run_cmd_exit_failure("./tools/scripts/ground/run_env.sh")

    print_update("Run \"./tools/scripts/ground/exec_interactive.sh /bin/bash\" in another shell", msg_type="SUCCESS")

    # Run until ^C
    run_cmd_exit_failure(GROUND_DOCKER_EXEC_SCRIPT + "'sleep infinity'")

    kill_ground()


def run_ground_shell(args):
    shutdown_functions.append(kill_ground)

    # Ground server and interface.
    print_update("Starting the Ground Station Docker container...")
    run_cmd_exit_failure("./tools/scripts/ground/run_env.sh")

    # Run interactive command line
    processes.run_command("./tools/scripts/ground/exec_interactive.sh /bin/bash")

    kill_ground()


def run_controls_docker_start(args=None, show_complete=True):
    print_update("Making sure all the necessary packages are installed")
    run_install()

    # Start the UAS@UCLA software development docker image if it is not already
    # running.
    run_env(show_complete=False)

    if show_complete:
        print_update("\n\nControls docker container started successfully", \
                msg_type="SUCCESS")


def run_controls_docker_rebuild(args=None, show_complete=True):
    print_update("Rebuilding docker environment.")
    run_install()

    run_controls_docker_kill(False)

    # Start the UAS@UCLA software development docker image if it is not already
    # running.
    run_env(show_complete=False, rebuild=True)

    if show_complete:
        print_update("\n\nControls docker container started successfully", \
                msg_type="SUCCESS")


def run_controls_docker_kill(args=None, show_complete=True):
    result = kill_controls()

    if show_complete:
        if result == "":
            print_update("\n\nControls docker container didn't exist in the first place", \
                    msg_type="FAILURE")
        else:
            print_update("\n\nControls docker container killed successfully", \
                    msg_type="SUCCESS")


def run_controls_docker_shell(args):
    # Make sure the controls docker image is running first.
    run_controls_docker_start(None, show_complete=False)

    # Run interactive command line
    print_update("Starting shell tunnel to controls docker container")
    processes.run_command("./tools/scripts/controls/exec_interactive.sh /bin/bash")


def run_controls_test_rrtavoidance(args):
    shutdown_functions.append(kill_processes_in_uas_env_container)

    print_update("Testing rrt avoidance...")

    run_controls_build(show_complete=False)
    processes.run_command(DOCKER_EXEC_SCRIPT + CONTROLS_TEST_RRT_AVOIDANCE_SCRIPT)

    print_update("\n\nRRT avoidance tests passed!", \
            msg_type="SUCCESS")



def run_env(args=None, show_complete=True, rebuild=False):
    print_update("Starting UAS@UCLA development environment...")

    if rebuild:
        run_cmd_exit_failure(DOCKER_RUN_ENV_SCRIPT + " --rebuild")
    else:
        run_cmd_exit_failure(DOCKER_RUN_ENV_SCRIPT)

    if show_complete:
        print_update("UAS@UCLA development environment started " \
                "successfully!", msg_type="SUCCESS")


def run_nuke(args):
    run_cmd_exit_failure(NUKE_SCRIPT)

    print_update("Successfully nuked the UAS@UCLA development environment! " \
            ">:)", msg_type="SUCCESS")

def run_vscode(args):
    run_cmd_exit_failure("code ./vscode.code-workspace")

    print_update("Started vscode!", msg_type="SUCCESS")

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

    install_parser = subparsers.add_parser('install')
    install_parser.set_defaults(func=run_install)

    travis_parser = subparsers.add_parser('travis')
    travis_parser.set_defaults(func=run_travis)

    cleanup_docker_parser = subparsers.add_parser('cleanup_docker')
    cleanup_docker_parser.set_defaults(func=run_cleanup_docker)

    kill_dangling_parser = subparsers.add_parser('kill_dangling')
    kill_dangling_parser.set_defaults(func=run_kill_dangling)

    interop_parser = subparsers.add_parser('interop')
    interop_parser.set_defaults(func=run_interop)

    controls_parser = subparsers.add_parser('controls')
    controls_subparsers = controls_parser.add_subparsers()
    controls_docker_parser = controls_subparsers.add_parser('docker')
    controls_docker_subparsers = controls_docker_parser.add_subparsers()
    controls_docker_start = controls_docker_subparsers.add_parser('start')
    controls_docker_start.set_defaults(func=run_controls_docker_start)
    controls_docker_rebuild = controls_docker_subparsers.add_parser('rebuild')
    controls_docker_rebuild.set_defaults(func=run_controls_docker_rebuild)
    controls_docker_kill = controls_docker_subparsers.add_parser('kill')
    controls_docker_kill.set_defaults(func=run_controls_docker_kill)
    controls_docker_shell = controls_docker_subparsers.add_parser('shell')
    controls_docker_shell.set_defaults(func=run_controls_docker_shell)
    controls_sitl_parser = controls_subparsers.add_parser('sitl')
    controls_sitl_parser.set_defaults(func=run_controls_sitl)
    controls_simulate_parser = controls_subparsers.add_parser('simulate')
    controls_simulate_parser.add_argument('--lite', action='store_true')
    controls_simulate_parser.set_defaults(func=run_controls_simulate)
    controls_build_parser = controls_subparsers.add_parser('build')
    controls_build_parser.set_defaults(func=run_controls_build)
    controls_deploy_parser = controls_subparsers.add_parser('deploy')
    controls_deploy_parser.set_defaults(func=run_controls_deploy)
    controls_ros_parser = controls_subparsers.add_parser('rqt')
    controls_ros_parser.set_defaults(func=run_controls_rqt)
    controls_test_parser = controls_subparsers.add_parser('test')
    controls_test_subparsers = controls_test_parser.add_subparsers()
    controls_test_all = controls_test_subparsers.add_parser('all')
    controls_test_all.set_defaults(func=run_unittest)
    controls_test_rrtavoidance_parser = controls_test_subparsers.add_parser('rrt_avoidance')
    controls_test_rrtavoidance_parser.set_defaults(func=run_controls_test_rrtavoidance)

    ground_parser = subparsers.add_parser('ground')
    ground_subparsers = ground_parser.add_subparsers()
    ground_docker_parser = ground_subparsers.add_parser('docker')
    ground_docker_parser.set_defaults(func=run_ground_docker)
    ground_build_parser = ground_subparsers.add_parser('build')
    ground_build_parser.set_defaults(func=run_ground_build)
    ground_build_parser.add_argument('ground_args', nargs='*')
    ground_run_parser = ground_subparsers.add_parser('run')
    ground_run_parser.add_argument('ground_args', nargs='*')
    ground_run_parser.set_defaults(func=run_ground_run)
    ground_shell_parser = ground_subparsers.add_parser('shell')
    ground_shell_parser.set_defaults(func=run_ground_shell)

    vision_parser = subparsers.add_parser('vision', help='vision help')
    vision_subparsers = vision_parser.add_subparsers()
    vision_subparsers.add_parser('build').set_defaults(func=run_vision_build)
    vision_run_parser = vision_subparsers.add_parser('run')
    vision_run_parser.set_defaults(func=run_vision)
    vision_run_parser.add_argument('vision_args', nargs=argparse.REMAINDER)

    unittest_parser = subparsers.add_parser('unittest')
    unittest_parser.set_defaults(func=run_unittest)

    run_env_parser = subparsers.add_parser('run_env')
    run_env_parser.set_defaults(func=run_env)

    jenkins_server_parser = subparsers.add_parser('jenkins_server')
    jenkins_server_parser.set_defaults(func=run_jenkins_server)

    jenkins_client_parser = subparsers.add_parser('jenkins_client')
    jenkins_client_parser.set_defaults(func=run_jenkins_client)
    jenkins_client_parser.add_argument('--auth', action='store', required=True)

    lint_parser = subparsers.add_parser('lint')
    lint_parser.set_defaults(func=run_lint)
    lint_parser.add_argument('--format', action='store_true')
    lint_parser.add_argument('--check', action='store_true')

    nuke_parser = subparsers.add_parser('nuke')
    nuke_parser.set_defaults(func=run_nuke)

    vscode_parser = subparsers.add_parser('vscode')
    vscode_parser.set_defaults(func=run_vscode)

    help_parser = subparsers.add_parser('help')
    help_parser.set_defaults(func=run_help)

    args = parser.parse_args()
    args.func(args)
