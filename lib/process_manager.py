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
            show_output=True, allow_input=True):
        cwd = self.get_cwd(rel_cwd)
        devnull = open(os.devnull, 'wb')

        if show_output:
            if allow_input:
                proc = subprocess.Popen(command,
                        shell=True, \
                        preexec_fn=os.setsid, \
                        cwd=cwd)
            else:
                proc = subprocess.Popen(command,
                        shell=True, \
                        preexec_fn=os.setsid, \
                        cwd=cwd, \
                        stdin=devnull)
        else:
            if allow_input:
                proc = subprocess.Popen(command,
                        shell=True, \
                        preexec_fn=os.setsid, \
                        cwd=cwd,
                        stdout=devnull,
                        stderr=devnull)
            else:
                proc = subprocess.Popen(command,
                        shell=True, \
                        preexec_fn=os.setsid, \
                        cwd=cwd, \
                        stdin=devnull, \
                        stdout=devnull, \
                        stderr=devnull)
        if track:
            self.procs.append(proc)

    def spawn_process_wait_for_code(self, command, show_output=True, \
            allow_input=True):
        devnull = open(os.devnull, 'wb')

        if show_output:
            if allow_input:
                proc = subprocess.Popen(command,
                        shell=True, \
                        preexec_fn=os.setsid)
            else:
                proc = subprocess.Popen(command,
                        shell=True, \
                        preexec_fn=os.setsid, \
                        stdin=devnull)
        else:
            if allow_input:
                proc = subprocess.Popen(command,
                        shell=True, \
                        preexec_fn=os.setsid, \
                        stdout=devnull, \
                        stderr=devnull)
            else:
                proc = subprocess.Popen(command,
                        shell=True, \
                        preexec_fn=os.setsid, \
                        stdin=devnull, \
                        stdout=devnull, \
                        stderr=devnull)
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
        killed_status = ""

        for proc in self.procs:
            # Continuously send interrupt signal until process exits.
            start = time.time()

            process_already_killed = False

            while True:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except:
                    process_already_killed = True
                    pass

                if proc.poll() == None:
                    time.sleep(0.1)
                else:
                    break

            if not process_already_killed:
                killed_status += "Killed pid #" + str(proc.pid) + " in " \
                        + "%.2f" % (time.time() - start) + " seconds.\n"

        return killed_status

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
