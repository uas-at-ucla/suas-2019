import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../util')

import process_manager

process_manager.ProcessManager().run_command("docker run --rm --interactive --tty\
    --publish 8000:80 --name interop-server auvsisuas/interop-server", \
    rel_cwd = "interop/server")