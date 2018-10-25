import os
import sys
import argparse
import subprocess

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, 'tools')
import npm_install

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--build', action='store_true')
    parser.add_argument('--server', action='store_true')
    parser.add_argument('--server', action='store_true')
    args = parser.parse_args()

    os.chdir("server")
    npm_install.npm_install()
    os.chdir("..")

    os.chdir("ui")
    npm_install.npm_install()
    os.chdir("..")

    # Run only if the user did not specify to just build the code.
    if not args.build:
        os.chdir("server")
        subprocess.call(["npm", "start"])
        os.chdir("..")
