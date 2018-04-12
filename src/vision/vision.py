import os
import sys
import signal
import argparse
import threading
import queue
from argparse import Namespace
from flask import Flask, render_template
import flask_socketio, socketIO_client
from darkflow.net.build import TFNet

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

processes = process_manager.ProcessManager()

verbose = False

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
    vision_socketio_server.emit('rsync', {'file_path': json['file_path']})


@vision_socketio_server.on('download_complete')
def call_yolo(json):
    print('Download Complete -> Telling yolo client to run on the image')
    vision_socketio_server.emit('yolo', {'file_path': json['local_file_path']})


def server_worker(args):
    vision_socketio_server.run(socketio_app, '0.0.0.0', port=int(args.port))


# Clients ######################################################################
vision_client = None
work_queue = queue.Queue()

def client_add_task(*args):
    work_queue.put(args)

def client_worker(args, worker_class):
    # parse ip and port
    ip_and_port = args.vision_srv.split(':')
    ip = ip_and_port[0]
    port = int(ip_and_port[1])

    # Connect to vision server
    print('Attempting to connect to server @ ' + ip + str(port))
    global vision_client
    vision_client = socketIO_client.SocketIO(ip, port=port)
    print('Connected to server!')

    # initialize worker and listen for tasks
    global work_queue
    client_worker = worker_class(work_queue, args)
    vision_client.on(client_worker.get_name(), client_add_task)
    vision_client.wait()

class ClientWorker(threading.Thread):
    def __init__(self, in_q, args): # accept args anyway even if not used
        super()
        self.in_q = in_q # input queue (queue.Queue)
        self.stop_req = threading.Event() # listen for a stop request

    def _do_work(self, task):
        raise NotImplementedError("Client Worker doesn't know what to do.")

    def get_name(self):
        raise NotImplementedError("Client Worker needs a name.")

    def run(self):
        while not self.stop_req.isSet(): # Exit run if stop was requested
            try:
                # Try to get an item from the queue
                # blocking: true; timeout: 0.05
                task = self.in_q.get(True, 0.05)
                self._do_work(task)
            except queue.Queue.Empty:
                continue

    def join(self, timeout=None):
        self.stop_req.set()
        super().join(timeout)

# Rsync file synchronization ###################################################
class RsyncWorker(ClientWorker):
    # task format: [user, addr, img_remote_src, img_local_dest]
    def _do_work(self, task):
        processes.spawn_process(
            'rsync -vz --progress -e "ssh -p 22" "' + task[0] + '@' +
            task[1] + ':' + task[2] + '" ' + task[3])
        if verbose:
            print('Called rsync with args: <' + '> <'.join(map(str, task)) + '>')
        # TODO emit proper response
        vision_client.emit('download_complete',
                           {'local_file_path': 'dummy_local_path'})

    def get_name(self):
        return 'rsync'

def rsync_worker(args):
    client_worker(args, RsyncWorker)

# YOLO image classification ####################################################
class YoloWorker(ClientWorker):
    def __init__(self, in_q, args):
        super().__init__(in_q, args)

        # load model
        yolo_options = {
            "model": "cfg/yolo-auvsi.cfg",
            "load": "bin/tiny-yolo-voc.weights",
            "threshold": 0.1
        }
        self.tfnet = TFNet(options)
        
    # task format: [filename]
    def _do_work(self, task):
        if verbose:
            print('Called yolo with args: <' + '> <'.join(map(str, task)) + '>')
        
        # TODO check if file is not local, and download (rsync) if necessary
        if not os.path.isfile(task[0]):
            pass

        img = cv2.imread(task[0])
        results = self.tfnet.return_predict(img)
        vision_client.emit('yolo_done', {'img_processed': task[0], 'results': results})
        
    def get_name(self):
        return 'yolo'

def yolo_worker(args):
    client_worker(args, YoloWorker)

# Parse command line arguments #################################################
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    parser = argparse.ArgumentParser()
    parser.add_argument("--verbose", action='store_true')

    subparsers = parser.add_subparsers(help='sub-command help')

    # Server Parser ####################################################
    server_port = None
    server_parser = subparsers.add_parser('server', help='server help')
    server_parser.add_argument(
        "--port", action='store', dest='port', required=True)
    server_parser.set_defaults(func=server_worker)

    # Client Parsers ###################################################
    client_parser = subparsers.add_parser('client', help='client help')
    client_parser.add_argument(
        "--vision-srv", action='store', dest='vision_srv', required=True)
    client_subparsers = client_parser.add_subparsers()

    # Rsync specific args
    rsync_parser = client_subparsers.add_parser('rsync', help='rsync help')
#    rsync_parser.add_argument(
#        "--img-src-dir", action='store', dest='img_src_dir', required=True)
#    rsync_parser.add_argument(
#        "--addr", action='store', dest='addr', required=True)
#    rsync_parser.add_argument(
#        "--user", action='store', dest='user', required=True)
#    rsync_parser.add_argument(
#        "--img-dest-dir", action='store', dest='img_dest_dir', required=True)
    rsync_parser.set_defaults(func=rsync_worker)

    # Yolo specific args
    yolo_parser = subparsers.client_subparsers('yolo', help='yolo help')
    yolo_parser.set_defaults(func=yolo_worker)

    args = parser.parse_args()
    global verbose
    verbose = args.verbose
    args.func(args)
