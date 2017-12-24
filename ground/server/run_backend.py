import os

################################################################################
#                                                                              #
#           NOTE: ALL ERROR OUTPUT IS BEING PIPED TO /dev/null                 #
#                 IF THIS CODE HAS ERRORS, THEN THEY WILL NOT APPEAR!          #
#                                                                              #
################################################################################
import sys
sys.stderr = open('/dev/null', 'w')

# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

RUN_INTEROP = True

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

if RUN_INTEROP:
    processes.spawn_process("python run_interop.py")
processes.spawn_process("python feed_interface.py")

signal.pause()
