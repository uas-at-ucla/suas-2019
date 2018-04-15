import os
import sys
import signal
import argparse
from argparse import Namespace
from flask import Flask, render_template
import flask_socketio, socketIO_client

# Yolo dependencies
from darkflow.net.build import TFNet
import cv2

# Multithreading
import threading
import queue

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

# Defaults #####################################################################
SECRET_KEY = 'flappy'
DEFAULT_SRV_IP = '0.0.0.0'
DEFAULT_SRV_PORT = 8099
DEFAULT_VSN_SRV = DEFAULT_SRV_IP + ':' + str(DEFAULT_SRV_PORT)
DEFAULT_THREADS = 1
MAX_THREADS = 20
DEFAULT_YOLO_CFG = 'localizer/cfg/yolo-auvsi.cfg'
DEFAULT_YOLO_WEIGHTS = 'localizer/darkflow/bin/tiny-yolo-voc.weights'
DEFAULT_YOLO_THRESH = 0.1

processes = process_manager.ProcessManager()
c_workers = []

verbose = False


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()
    # Ask the workers to join us in death
    for worker in c_workers:
        worker.join()
    sys.exit(0)


# Server #######################################################################
socketio_app = Flask(__name__)
socketio_app.config['SECRET_KEY'] = SECRET_KEY
vision_socketio_server = flask_socketio.SocketIO(socketio_app)
img_count = 0

@vision_socketio_server.on('connect')
def vision_socketio_server_connect():
    print("Someone connected to vision server")


# Step 1 - download the image
@vision_socketio_server.on('process_image')
def process_image(json):
    print("Telling rsync client to download image")
    # TODO configure drone ip addr
    vision_socketio_server.emit('rsync', {'prev': 'null', 'next': 'null', 'user': 'pi', 'addr': 'INSERT_DRONE_IP', 'img_remote_src': json['file_path'], 'img_local_dest': ''})

# Step 2 - find the targets in the image (yolo)
@vision_socketio_server.on('download_complete')
def call_yolo(json):
    print("Telling yolo to run on img")
    vision_socketio_server.emit('yolo', {})

# Step 3 - cut out those targets
@vision_socketio_server.on('yolo_done')
def snip_img(json):
    print('Yolo finished -> ')

# Step 4 - run shape classification on each target

# Step 5 ... do letter classification and color recognition


# TODO handle autodownloading images as needed
#@vision_socketio_server.on('download_complete')
#def do_next_task(json):
#    if json['next_task'] == 'echo':
#        vision_socketio_server.emit('download_complete', json['local_file_path'])
#    vision_socketio_server.emit(json['next_task']['event_name'], {'file_path': json['local_file_path']})
#
#@vision_socketio_server.on('download_failed')
#def handle_download_failed(json):
#    # TODO handle download failure
#    pass


def server_worker(args):
    global img_count
    img_count = len(os.listdir(LOCAL_RAW_DIR))
    vision_socketio_server.run(socketio_app, '0.0.0.0', port=int(args.port))


# Clients #######################################################################
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
    for i in range(0, args.threads):
        c_worker = worker_class(work_queue, args)
        c_worker.start()
        c_workers.append(c_worker)
    vision_client.on(c_workers[0].get_event_name(), client_add_task)
    vision_client.wait()


class ClientWorker(threading.Thread):
    def __init__(self, in_q, args):  # accept args anyway even if not used
        super()
        self.in_q = in_q  # input queue (queue.Queue)
        self.stop_req = threading.Event()  # listen for a stop request

    def _do_work(self, task):
        raise NotImplementedError("Client Worker doesn't know what to do.")

    def get_event_name(self):
        raise NotImplementedError("Client Worker needs a name.")

    def run(self):
        while not self.stop_req.isSet():  # Exit run if stop was requested
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
    # task format: [prev, next, user, addr, img_remote_src, img_local_dest]
    def _do_work(self, task):
        if verbose:
            print(
                'Called rsync with args: <' + '> <'.join(map(str, task)) + '>')

        if 0 == processes.spawn_process_wait_for_code(
                'rsync -vz --progress -e "ssh -p 22" "' + task[2] + '@' +
                task[3] + ':' + task[4] + '" ' + task[5]):
            vision_client.emit('download_complete', {'saved_path': task[3], 'next_task': task[1]})
        else:
            vision_client.emit('download_failed', {'attempted_path': task[3], 'prev_task': task[0]})

    def get_event_name(self):
        return 'rsync'

def rsync_worker(args):
    client_worker(args, RsyncWorker)


# YOLO image classification ####################################################
class YoloWorker(ClientWorker):
    def __init__(self, in_q, args):
        super().__init__(in_q, args)

        # load model
        yolo_options = {
            "model": args.yolo_cfg,
            "load": args.yolo_weights,
            "threshold": args.yolo_threshold
        }
        self.tfnet = TFNet(options)

    # task format: [filename]
    def _do_work(self, task):
        if verbose:
            print(
                'Called yolo with args: <' + '> <'.join(map(str, task)) + '>')

        # TODO check if file is not local, and download (rsync) if necessary
        while not os.path.isfile(task[0]):
            vision_client.emit('rsync', {'next_task': 'echo'})

        img = cv2.imread(task[0])
        results = self.tfnet.return_predict(img)

        # TODO Calculate the coordinates
#        for result in results:
#            result
        vision_client.emit('yolo_done', {
            'img_processed': task[0],
            'results': results
        })

        # Result format: [{'label': str, 'confidence': int,
        #                  'topleft': {'x': int, 'y': int},
        #              'bottomright': {'x': int, 'y': int},
        #                   'coords': {'lat': float, 'lng': float}}]


    def get_event_name(self):
        return 'yolo'


def yolo_worker(args):
    client_worker(args, YoloWorker)

# Target Localization ##########################################################
class SnipperWorker(ClientWorker):

    # task format: [img, yolo_results, out_dir]
    def _do_work(self, task):
        yolo_results = task[1]
            

    def get_event_name(self):
        return 'snipper'

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
        "--port", action='store', dest='port', default=DEFAULT_SRV_PORT)
    server_parser.set_defaults(func=server_worker)

    # Client Parsers ###################################################
    client_parser = subparsers.add_parser('client', help='client help')
    client_parser.add_argument(
        "--vision-srv",
        action='store',
        dest='vision_srv',
        default=DEFAULT_VSN_SRV)
    client_parser.add_argument(
        "--threads",
        action='store',
        dest='threads',
        default=DEFAULT_THREADS,
        choices=range(1, MAX_THREADS + 1))

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
    yolo_parser.add_argument(
        "--cfg", action='store', dest='yolo_cfg', default=DEFAULT_YOLO_CFG)
    yolo_parser.add_argument(
        "--weights",
        action='store',
        dest='yolo_weights',
        default=DEFAULT_YOLO_WEIGHTS)
    yolo_parser.add_argument(
        "--thresh",
        action='store',
        dest='yolo_threshold',
        default=DEFAULT_YOLO_THRESH)
    yolo_parser.set_defaults(func=yolo_worker)

    args = parser.parse_args()
    global verbose
    verbose = args.verbose
    args.func(args)
