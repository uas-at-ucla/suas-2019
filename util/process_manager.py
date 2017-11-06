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

    def spawn_process(self, command, rel_cwd=None, track=True):
        cwd = self.get_cwd(rel_cwd)
        proc = subprocess.Popen(command, \
                                shell = True, \
                                preexec_fn = os.setsid, \
                                cwd = cwd)
        if track:
            self.procs.append(proc)

    def killall(self):
        for proc in self.procs:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)

    def run_command(self, command, rel_cwd=None):
        cwd = self.get_cwd(rel_cwd)
        subprocess.call(command, shell = True, cwd = cwd)

    def get_cwd(self, rel_path):
        if rel_path is not None:
            return os.path.realpath(os.path.join(os.getcwd(), rel_path))
        return None
