import sys
import os
import signal
import time

# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

sys.path.insert(0, '../util')
import process_manager

processes = None

def signal_received(signal, frame):
    processes.killall()
    sys.exit(0)

if __name__ == "__main__":
    processes = process_manager.ProcessManager()

    processes.run_command("npm run build", rel_cwd="client")
    processes.spawn_process("python server/run_interop.py")
    processes.spawn_process("python server/run_ground.py")

    signal.signal(signal.SIGINT, signal_received)

    signal.pause()
