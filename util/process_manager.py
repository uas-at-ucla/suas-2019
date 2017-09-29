import sys
sys.dont_write_bytecode = True
import signal
import subprocess

# This class assists in spawning multiple processes and terminating all these
# processes when a program exits.
class ProcessManager:
    def __init__(self):
        self.procs = list()
        signal.signal(signal.SIGINT, self.terminate_all)

    def spawn_process(self, command):
        try:
            proc = subprocess.Popen(command, shell=True)
            self.procs.append(proc)
        except:
            print "FAILED " + command
            pass

    def terminate_all(self, signal, frame):
        print "Attempting to terminate all spawned processes."
        for proc in self.procs:
            proc.send_signal(1)
        exit(0)
