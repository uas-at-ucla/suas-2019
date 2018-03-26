import os
import sys
import signal
import argparse
from flask import Flask, render_template
import flask_socketio, socketIO_client

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

processes = process_manager.ProcessManager()

def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()
    sys.exit(0)

# Server #######################################################################
socketio_app = Flask(__name__)
socketio_app.config['SECRET_KEY'] = 'flappy'
vision_socketio_server = flask_socketio.SocketIO(socketio_app)
@vision_socketio_server.on('connect')
def vision_socketio_server_connect():
    print("Someone connected to vision server")

def server_worker(args):
    vision_socketio_server.run(socketio_app, '0.0.0.0', port=int(args.port))

# Rsync file synchronization ###################################################
def rsync_worker(args):
    # rsync -avz --progress -e "ssh -p 22" "user@server.org:~/file.cc" .
    print(args.src)

# YOLO image classification ####################################################
def yolo_worker(args):
    pass

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(help='sub-command help')

    server_port = None
    server_parser = subparsers.add_parser('server', help='server help')
    server_parser.add_argument("--port", action='store', dest='port',
            required=True)
    server_parser.set_defaults(func=server_worker)

    rsync_url = None
    rsync_parser = subparsers.add_parser('rsync', help='rsync help')
    rsync_parser.add_argument("--src", action='store', dest='src',
            required=True)
    rsync_parser.set_defaults(func=rsync_worker)

    yolo_parser = subparsers.add_parser('yolo', help='yolo help')
    yolo_parser.set_defaults(func=yolo_worker)

    args = parser.parse_args()
    args.func(args)
