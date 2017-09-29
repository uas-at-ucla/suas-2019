import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../util')
import time
import signal

import process_manager

def main():
	# Initialize a process manager to keep track of all spawned processes and
	# kill all these processes on exit.
    test_drone = process_manager.ProcessManager()
    signal.signal(signal.SIGINT, test_drone.terminate_all)

    # Wait for program to terminate
    signal.pause()

if __name__ == "__main__":
    main()
