import os
import sys
import argparse
import subprocess

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, 'tools')
import npm_install

def build():
    os.chdir("server")
    npm_install.npm_install()
    os.chdir("..")

    os.chdir("ui")
    npm_install.npm_install()
    os.chdir("..")

    print("done building")

def run_all(args):
    # run start-web (w/o Electron) if web option specified. Otherwise, run start (w/ Electron).
    subprocess.call(["npm", "run", "start"+args.web], cwd="server")

def run_server(args):
    subprocess.call(["node", "ground_server.js"], cwd="server")

def run_ui(args):
    # run start-web (w/o Electron) if web option specified. Otherwise, run start (w/ Electron).
    subprocess.call(["npm", "run", "start"+args.web], cwd="ui")

def deploy_win(args):
    subprocess.call(["npm", "run", "package-win"], cwd="ui")

def deploy_mac(args):
    subprocess.call(["npm", "run", "package-mac"], cwd="ui")

def deploy_linux(args):
    subprocess.call(["npm", "run", "package-linux"], cwd="ui")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='option')
    subparsers.required = True
    subparsers.add_parser('build').set_defaults(func=lambda args: None) # do nothing

    run_parser = subparsers.add_parser('run')
    run_parser.set_defaults(func=run_all)
    run_parser.add_argument('--web', action='store_const', const="-web", default="")
    run_subparsers = run_parser.add_subparsers()
    
    run_subparsers.add_parser('all')
    run_subparsers.add_parser('server').set_defaults(func=run_server)
    run_subparsers.add_parser('ui').set_defaults(func=run_ui)

    deploy_win_parser = subparsers.add_parser('deploy-win')
    deploy_win_parser.set_defaults(func=deploy_win)

    deploy_mac_parser = subparsers.add_parser('deploy-mac')
    deploy_mac_parser.set_defaults(func=deploy_mac)

    deploy_linux_parser = subparsers.add_parser('deploy-linux')
    deploy_linux_parser.set_defaults(func=deploy_linux)

    args = parser.parse_args()
    build() # always build
    args.func(args)