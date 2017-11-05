import sys
sys.dont_write_bytecode = True

import subprocess
import os
import signal

# This class assists in spawning multiple processes and terminating all these
# processes when a program exits.
class ProcessManager:
    def __init__(self):
        self.procs = list()

    def spawn_process(self, command, cwd=None, track=True):
        if cwd is not None:
            cwd = os.path.realpath(os.path.join(os.getcwd(), cwd))
        proc = subprocess.Popen(command, \
                                shell = True, \
                                preexec_fn = os.setsid, \
                                cwd = cwd)
        if track:
            self.procs.append(proc)

    def killall(self):
        for proc in self.procs:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)

    def run_command(self, command):
        subprocess.call(command, shell = True)
