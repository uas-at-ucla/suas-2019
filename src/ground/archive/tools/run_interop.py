import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../../../lib')

import signal
import process_manager

os.chdir("../interop/server")

interop_processes = None


def signal_received(signal, frame):
    interop_processes.killall()
    interop_processes.run_command("docker kill interop-server")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_received)

# First kill the interop server. We don't want a previous instance
# running. If it is not running (the usual case), an error is returned,
# which can be ignored.
interop_processes = process_manager.ProcessManager()
interop_processes.run_command("docker rm -f interop-server")

print("SPAWN interop")
interop_processes.spawn_process("docker run " +\
                                  "--rm " + \
                                  "--publish 8000:80 " + \
                                  "--name interop-server " + \
                                  "auvsisuas/interop-server")

signal.pause()
