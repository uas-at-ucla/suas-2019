import os
import sys

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

processes = process_manager.ProcessManager()

def run_command(cmd):
    processes.spawn_process_wait_for_code(cmd)

# TODO: Handle command line arguments
# print(sys.argv)

run_command("cd server; ../tools/npm_install.sh")
run_command("cd ui; ../tools/npm_install.sh")

processes.run_command("cd server; npm start")