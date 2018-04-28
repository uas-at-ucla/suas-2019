import sys
sys.dont_write_bytecode = True

import subprocess
import os
import signal
import time

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
                    shell=True, \
                    preexec_fn=os.setsid, \
                    cwd=cwd)
        else:
            devnull = open(os.devnull, 'wb')
            proc = subprocess.Popen(command,
                    shell=True, \
                    preexec_fn=os.setsid, \
                    cwd=cwd,
                    stdout=devnull,
                    stderr=devnull)
        if track:
            self.procs.append(proc)

    def spawn_process_wait_for_code(self, command):
        proc = subprocess.Popen(command,
                shell=True, \
                preexec_fn=os.setsid)
        self.procs.append(proc)

        proc.communicate()
        out = proc.returncode
        return out

    def wait_for_complete(self):
        for proc in self.procs:
            while True:
                if proc.poll() == None:
                    time.sleep(0.1)
                else:
                    break

    def killall(self):
        for proc in self.procs:
            # Continuously send interrupt signal until process exits.
            while True:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except:
                    pass
                if proc.poll() == None:
                    time.sleep(0.1)
                else:
                    break

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
