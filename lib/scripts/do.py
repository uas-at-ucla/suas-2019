import os
import sys
import signal
import time
import argparse

os.chdir(os.path.dirname(os.path.realpath(__file__)))
os.chdir("../../")
sys.dont_write_bytecode = True
sys.path.insert(0, 'lib')
import process_manager

processes = process_manager.ProcessManager()

UAS_AT_UCLA_TEXT = '\033[94m' + \
' ___  ___  ________  ___       ________      ___  ___  ________  ________\n' + \
'|\  \|\  \|\   ____\|\  \     |\   __  \    |\  \|\  \|\   __  \|\   ____\ \n' + \
'\ \  \\\\\  \ \  \___|\ \  \    \ \  \|\  \   \ \  \\\\\  \ \  \|\  \ \  \___|_\n' + \
' \ \  \\\\\  \ \  \    \ \  \    \ \   __  \   \ \  \\\\\  \ \   __  \ \_____  \ \n' + \
'  \ \  \\\\\  \ \  \____\ \  \____\ \  \ \  \   \ \  \\\\\  \ \  \ \  \|____|\  \ \n' + \
'   \ \_______\ \_______\ \_______\ \__\ \__\   \ \_______\ \__\ \__\____\_\  \ \n' + \
'    \|_______|\|_______|\|_______|\|__|\|__|    \|_______|\|__|\|__|\_________\ \n' + \
'                                                                   \|_________| \n' + \
'\n' + \
'#################################### do #######################################\n' + \
'\033[0m'


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()
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

    processes.spawn_process("sudo tools/installation/install.sh")
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


def run_build(args):
    run_and_die_if_error("bazel build //src/...")
    run_and_die_if_error("bazel build //lib/...")
    run_and_die_if_error("bazel build //aos/linux_code:core")
    run_and_die_if_error("bazel build --cpu=raspi //src/...")
    run_and_die_if_error("bazel build --cpu=raspi //aos/linux_code:core")


def run_simulate(args):
    processes.spawn_process("bazel build //src/...")
    processes.wait_for_complete()
    processes.spawn_process("bazel build @PX4_sitl//:jmavsim")
    processes.wait_for_complete()

    # Initialize shared memory for queues.
    processes.spawn_process("ipcrm --all", None, True, args.verbose)
    processes.wait_for_complete()

    processes.spawn_process("./lib/scripts/bazel_run.sh //aos/linux_code:core",
                            None, True, args.verbose)

    # Give aos core some time to run.
    time.sleep(0.5)

#   # Simulator and port forwarder.
    processes.spawn_process("./lib/scripts/bazel_run.sh @PX4_sitl//:jmavsim",
                            None, True, args.verbose)
    processes.spawn_process("mavproxy.py " \
            "--mav20 " \
            "--master=0.0.0.0:14550 " \
            "--out=udp:0.0.0.0:8083 " \
            "--out=udp:0.0.0.0:8085 ", \
            None, True, args.verbose)

    # Log writer.
    processes.spawn_process("./bazel-out/k8-fastbuild/bin/lib/logger/" \
            "log_writer");

    # Drone control code.
    processes.spawn_process(
            "./bazel-out/k8-fastbuild/bin/src/control/ground_communicator/" \
                    "ground_communicator", None, True, True)
    processes.spawn_process("./bazel-out/k8-fastbuild/bin/src/control/io/io",
                            None, True, args.verbose)
    processes.spawn_process(
        "./bazel-out/k8-fastbuild/bin/src/control/loops/flight_loop", None,
        True, args.verbose)

    # Ground server and interface.
    processes.spawn_process("python ./src/ground/ground.py", None, True, True)
    processes.wait_for_complete()


def run_ground(args):
    # Ground server and interface.
    processes.spawn_process("python ./src/ground/ground.py", None, True,
                            args.verbose)
    processes.wait_for_complete()


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
    ground_parser.set_defaults(func=run_ground)

    build_parser = subparsers.add_parser('build', help='build help')
    build_parser.set_defaults(func=run_build)

    args = parser.parse_args()
    args.func(args)
