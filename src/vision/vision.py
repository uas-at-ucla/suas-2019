import os
import sys
import signal
import argparse
from argparse import Namespace
from flask import Flask, render_template
import flask_socketio, socketIO_client

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

processes = process_manager.ProcessManager()
task_queue = []

# class Task:
#     def execute(self):
#         raise NotImplementedError()

# class DownloadImageTask(Task):
#     def __init__(self, image_path):
#         self._image_path = image_path
#     def execute(self):
#         # 

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

@vision_socketio_server.on('process_image')
def process_image(json):
    print("Telling rsync clients to download image")
    vision_socketio_server.emit('rsync', {'file_path':json['file_path']})

@vision_socketio_server.on('download_complete')
def call_yolo(json):
    print('Download Complete -> Telling yolo client to run on the image')
    vision_socketio_server.emit('yolo', {'file_path':json['local_file_path']})

def server_worker(args):
    vision_socketio_server.run(socketio_app, '0.0.0.0', port=int(args.port))

# Clients ######################################################################
vision_client = None

def connect_to_server(ip, port):
    global vision_client
    vision_client = socketIO_client.SocketIO(ip, port=port)

# Rsync file synchronization ###################################################
def rsync_worker(args):
    # rsync -avz --progress -e "ssh -p 22" "user@server.org:~/file.cc" .
    ip_and_port = args.vision_srv.split(':')
    ip = ip_and_port[0]
    port = int(ip_and_port[1])
        
    print('Attempting to connect to server @ ' + ip + str(port))
    connect_to_server(ip, port)
    print('Connected to server!')
    vision_client.on('rsync', run_rsync)
    vision_client.wait()
    
def run_rsync(*args):
    # TODO Use rsync to download the image with the given filepath
    # processes.spawn_process('something with rsync')
    print('Called rsync with args: <' + '> <'.join(map(str, args)) + '>')
    vision_client.emit('download_complete', {'local_file_path': 'dummy_local_path'})

# YOLO image classification ####################################################
def yolo_worker(args):
    ip_and_port = args.vision_srv.split(':')
    ip = ip_and_port[0]
    port = int(ip_and_port[1])

    print('Attempting to connect to server @ ' + ip + str(port))
    connect_to_server(ip, port)
    print('Connected to server!')
    vision_client.on('yolo', run_yolo)
    vision_client.wait()

def run_yolo(*args):
    # TODO Use yolo to identify the image with the given filepath and cut out the targets
    # processes.spawn_process('something with rsync')
    print('Called yolo with args: <' + '> <'.join(map(str, args)) + '>')
    vision_client.emit('targets_located', {'file_path': 'dummy_local_path', 'localized_targets_path': 'dummy_targets_path'})

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
    # rsync_parser.add_argument("--src", action='store', dest='src',
    #        required=True)
    rsync_parser.add_argument("--vision_srv", action='store', dest='vision_srv', required=True)
    rsync_parser.set_defaults(func=rsync_worker)

    yolo_parser = subparsers.add_parser('yolo', help='yolo help')
    rsync_parser.add_argument("--vision_srv", action='store', dest='vision_srv', required=True)
    yolo_parser.set_defaults(func=yolo_worker)

    args = parser.parse_args()
    args.func(args)
