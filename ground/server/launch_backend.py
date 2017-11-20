import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../../util')

import signal
import process_manager

processes = process_manager.ProcessManager()

def signal_received(signal, frame):
    processes.killall()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_received)

processes.spawn_process("python run_interop.py")
processes.spawn_process("python feed_interface.py")
# TODO: Figure out how to spawn npm start in a script (python or bash) and have it exit correctly.
# processes.spawn_process("npm start", rel_cwd="client")

signal.pause()
