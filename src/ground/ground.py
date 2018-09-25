import os
import sys

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

processes = process_manager.ProcessManager()

print("ground.py!!!")
print(sys.argv)

processes.spawn_process_wait_for_code("cd server && ../tools/npm_install.sh")
processes.spawn_process_wait_for_code("cd ui && ../tools/npm_install.sh")

print("ground.py is a work in progess!")