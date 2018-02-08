import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
dname = os.path.dirname(os.path.realpath(__file__))
os.chdir(dname)

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../../util')

import process_manager

processes = process_manager.ProcessManager()

with open("setup_mission.py") as f:
	content = f.read()

processes.run_command('docker exec interop-server bash -c "echo \\"' + content + '\\" > setup_mission.py"')

processes.run_command("docker exec interop-server python setup_mission.py")
