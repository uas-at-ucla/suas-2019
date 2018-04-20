import os
import sys
import signal
import argparse
from argparse import Namespace
from flask import Flask, render_template
import flask_socketio, socketIO_client

# Server dependencies
import time
import uuid

# Yolo dependencies
from darkflow.net.build import TFNet  # yolo neural net
import cv2  # reading images

# Multithreading
import threading  # multithreading library
import queue  # input/output queues

# Snipper dependencies
# cv2 imported above # cropping and saving images
import ntpath  # finding the name of a file (windows compatible)

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
DEFAULT_LOCAL_DATA_DIR = 'local_data'
DEFAULT_AUCTION_TIMEOUT = 3

processes = process_manager.ProcessManager()
c_workers = []
s_worker = None

verbose = False


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()
    # Ask the workers to join us in death
    s_worker.join()
    for worker in c_workers:
        worker.join()
    sys.exit(0)


# Server #######################################################################

socketio_app = Flask(__name__)
socketio_app.config['SECRET_KEY'] = SECRET_KEY
vision_socketio_server = flask_socketio.SocketIO(socketio_app)
server_task_queue = queue.Queue()
connected_clients = {'rsync': [], 'yolo': [], 'snipper': []}
active_auctions = {}
img_count = 0


class ServerWorker(threading.Thread):
    def __init__(self, in_q):  # accept args anyway even if not used
        super()
        self.in_q = in_q  # input queue (queue.Queue)
        self.stop_req = threading.Event()  # listen for a stop request

    def run(self):
        while not self.stop_req.isSet():  # Exit run if stop was requested
            try:
                # Try to get an item from the queue
                # blocking: true; timeout: 0.05
                task = self.in_q.get(True, 0.05)

                # check on the progress of an auction
                if task['type'] == 'timeout':
                    if (time.time() -
                            task['time_began']) >= DEFAULT_AUCTION_TIMEOUT:
                        auction = active_auctions[task['auction_id']]

                        # reset the timer if no bids have been made
                        if len(auction['bids']) == 0:
                            task['time_began'] = time.time()
                            self.in_q.push(task)
                        else:
                            # choose the lowest bidder
                            lowest_bid = auction['bids'][0]
                            for bid in auction['bids']:
                                if bid['workload'] < lowest_bid['workload']:
                                    lowest_bid = bid

                            # announce the winner of the auction
                            vision_socketio_server.emit(
                                auction['event_name'] + '_' +
                                lowest_bid['client_id'], auction['args'])

                            # remove the auction entry
                            active_auctions.pop(task['auction_id'])
                    else:
                        # check again later since not enough time passed
                        self.in_q.push(task)

                # create a new auction
                elif task['type'] == 'auction':
                    # generate a random auction id
                    auction_id = uuid.uuid4()

                    # announce the auction to all clients
                    vision_socketio_server.emit(task['event_name'],
                                                {'auction_id': auction_id})

                    # add a new auction listing
                    active_auctions[auction_id] = {
                        'bids': [],
                        'event_name': task['event_name'],
                        'args': task['args']
                    }

                    # check in on this auction at a later time
                    self.in_q.push({
                        'type': 'timeout',
                        'time_began': time.time(),
                        'auction_id': auction_id
                    })
            except queue.Queue.Empty:
                continue

    def join(self, timeout=None):
        self.stop_req.set()
        super().join(timeout)


@vision_socketio_server.on('connect')
def vision_socketio_server_connect():
    print("Someone connected to vision server")


#@vision_socketio_server.on('depart')
#def remove_client(json):
#    if verbose:
#        print('Removing ' + json['type'] + ' client: ' + json['id'])
#    connected_clients[json['type']].remove(json['id'])
#
#@vision_socketio_server.on('announce')
#def add_client(json):
#    if verbose:
#        print('Adding ' + json['type'] + ' client: ' + json['id'])
#    connected_clients[json['type']].append(json['id'])


@vision_socketio_server.on('bid')
def receive_bid(json):
    active_auctions[json['auction_id']]['bids'].append(json['bid'])


# Step 1 - download the image
@vision_socketio_server.on('process_image')
def process_image(json):
    print("Telling rsync client to download image")
    # TODO configure drone ip addr
    # yapf: disable
    vision_socketio_server.emit(
        'rsync', {
            'prev': None,
            'next': None,
            'user': 'pi',
            'addr': 'INSERT_DRONE_IP',
            'img_remote_src': json['file_path'],
            'img_local_dest':
            os.path.join(
                os.path.abspath('.'), DEFAULT_LOCAL_DATA_DIR, 'raw',
                str(img_count) + '.jpg')
        })
    # yapf: enable
    img_count += 1


# Step 2 - find the targets in the image (yolo)
@vision_socketio_server.on('download_complete')
def call_yolo(json):
    print("Telling yolo to run on img")
    vision_socketio_server.emit('yolo', {'file_path': json['saved_path']})


# Step 3 - cut out those targets
@vision_socketio_server.on('yolo_done')
def snip_img(json):
    print('Yolo finished -> running snipper')
    vision_socketio_server.emit('snip', {
        'src_img_path': json['img_processed'],
        'yolo_results': json['results']
    })


# Step 4 - run shape classification on each target
#      ... do letter classification and color recognition
@vision_socketio_server.on('snipped')
def classify(json):
    pass


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
    global s_worker
    s_worker = ServerWorker(server_task_queue)
    s_worker.start()
    vision_socketio_server.run(socketio_app, '0.0.0.0', port=int(args.port))


# Clients ######################################################################
vision_client = None
client_id = None
work_queue = queue.Queue()


def client_add_task(*args):
    work_queue.put(args)


def client_bid_for_task(*args):
    auction = args[0]
    vision_client.emit(
        'bid', {
            'auction_id': auction['auction_id'],
            'bid': {
                'client_id': client_id,
                'workload': work_queue.qsize()
            }
        })


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
    global client_id
    client_id = uuid.uuid4()
    global work_queue
    for i in range(0, args.threads):
        c_worker = worker_class(work_queue, args)
        c_worker.start()
        c_workers.append(c_worker)
    vision_client.on(c_workers[0].get_event_name(), client_bid_for_task)
    vision_client.on(c_workers[0].get_event_name() + '_' + client_id,
                     client_add_task)
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
    # task format: [{'prev': {}, 'next': {}, 'user': str, 'addr': str,
    #                'img_remote_src': str, 'img_local_dest': str}]
    def _do_work(self, task):
        task_args = task[0]
        if verbose:
            print(
                'Called rsync with args: <' + '> <'.join(map(str, task)) + '>')

        if 0 == processes.spawn_process_wait_for_code(
                'rsync -vz --progress -e "ssh -p 22" "' + task_args['user'] +
                '@' + task_args['addr'] + ':' + task_args['img_remote_src'] +
                '" ' + task_args['img_local_dest']):
            vision_client.emit(
                'download_complete', {
                    'saved_path': task_args['img_local_dest'],
                    'next_task': task_args['next']
                })
        else:
            vision_client.emit(
                'download_failed', {
                    'attempted_path': task_args['img_local_dest'],
                    'prev_task': task_args['prev']
                })

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

    # task format: [{'file_path': str}]
    def _do_work(self, task):
        file_path = task[0]['file_path']
        if verbose:
            print(
                'Called yolo with args: <' + '> <'.join(map(str, task)) + '>')

        # TODO check if file is not local, and download (rsync) if necessary
        while not os.path.isfile(file_path):
            vision_client.emit('rsync', {'next_task': 'echo'})

        img = cv2.imread(file_path)
        results = self.tfnet.return_predict(img)

        # TODO Calculate the coordinates
        #        for result in results:
        #            result
        vision_client.emit('yolo_done', {
            'img_processed': file_path,
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

    # task format: [{'src_img_path': str, 'yolo_results': [], 'out_dir'}]
    def _do_work(self, task):
        src_img_path = task[0]['src_img_path']
        src_img_label = ntpath.basename(src_img_path).split('.')[0]
        yolo_results = task[0]['yolo_results']
        out_dir = task[0]['out_dir']

        src_img = cv2.imread(src_img_path)
        cropped_imgs = []
        i = 0
        for result in yolo_results:
            xmin = result['topleft']['x']
            xmax = result['bottomright']['x']
            ymin = result['topleft']['y']
            ymax = result['bottomright']['y']

            cropped_img = src_img[ymin:ymax, xmin:xmax]
            out_path = os.path.join(out_dir,
                                    src_img_label + '-' + str(i) + '.jpg')
            cv2.imwrite(out_path, cropped_img)
            vision_client.emit('snipped', {'img_path': out_path})
            i += 1

    def get_event_name(self):
        return 'snip'


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
