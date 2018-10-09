import os
import sys
import argparse

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

processes = process_manager.ProcessManager()

def run_command(cmd):
    processes.spawn_process_wait_for_code(cmd)

# TODO: Handle command line arguments
# print(sys.argv)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args()

    run_command("cd server; ../tools/npm_install.sh")
    run_command("cd ui; ../tools/npm_install.sh")

    # Run only if the user did not specify to just build the code.
    if not args.build:
        processes.run_command("cd server; npm start")
