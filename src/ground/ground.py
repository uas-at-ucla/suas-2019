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
    subprocess.call(["npm", "start"], cwd="server")

def run_server(args):
    subprocess.call(["node", "ground_server.js"], cwd="server")

def run_ui(args):
    subprocess.call(["npm", "start"], cwd="ui")

def deploy_win(args):
    os.chdir("ui")
    os.system("npm run package-win")
    os.chdir("..")

def deploy_mac(args):
    os.chdir("ui")
    os.system("npm run package-mac")
    os.chdir("..")

def deploy_linux(args):
    os.chdir("ui")
    os.system("npm run package-linux")
    os.chdir("..")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()
    subparsers.add_parser('build').set_defaults(func=lambda args: None) # do nothing

    run_parser = subparsers.add_parser('run')
    run_parser.set_defaults(func=run_all)
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