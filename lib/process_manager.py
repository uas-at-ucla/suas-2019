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

    def spawn_process(self, command, rel_cwd=None, track=True, \
            show_output=True):
        cwd = self.get_cwd(rel_cwd)
        if show_output:
            proc = subprocess.Popen(command,
                    shell = True, \
                    preexec_fn = os.setsid, \
                    cwd = cwd)
        else:
            devnull = open(os.devnull, 'wb')
            proc = subprocess.Popen(command,
                    shell = True, \
                    preexec_fn = os.setsid, \
                    cwd = cwd,
                    stdout=devnull,
                    stderr=devnull)
        if track:
            self.procs.append(proc)

    def wait_for_complete(self):
        for proc in self.procs:
            os.waitpid(proc.pid, os.WNOHANG)

    def killall(self):
        for proc in self.procs:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except:
                pass

    def run_command(self, command, rel_cwd=None):
        cwd = self.get_cwd(rel_cwd)
        return subprocess.call(command, shell = True, cwd = cwd)

    def return_output(self, command, rel_cwd=None):
        cwd = self.get_cwd(rel_cwd)
        return subprocess.check_output(command, shell = True, cwd = cwd)

    def get_cwd(self, rel_path):
        if rel_path is not None:
            return os.path.realpath(os.path.join(os.getcwd(), rel_path))
        return None