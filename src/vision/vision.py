#!/bin/python3
import os
import sys
import signal
import argparse
from argparse import Namespace
from flask import Flask, render_template
import flask_socketio, socketIO_client
import logging

# Server dependencies
import time
import uuid
import json as json_module
import hashlib
import base64

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

# Yolo dependencies
sys.path.insert(0, './localizer')
from localizer.darkflow.net.build import TFNet  # yolo neural net
import cv2  # reading images

# Classification dependencies
sys.path.insert(0, './classifier')
from classifier import vision_classifier
import tensorflow as tf

# Multithreading
import threading  # multithreading library
import queue  # input/output queues

# Snipper dependencies
# cv2 imported above # cropping and saving images
import ntpath  # finding the name of a file (windows compatible)

# Client dependency
sys.path.insert(0, './util')
from util.img_manager import ImgManager


# Defaults #####################################################################

# General Defaults
DEFAULT_DATA_DIR = os.path.abspath('data_local')
print('cwd: ' + os.getcwd())
print('dir: ' + DEFAULT_DATA_DIR)

# Server defaults
DRONE_USER = 'benlimpa'
SECRET_KEY = 'flappy'
DEFAULT_SRV_IP = '0.0.0.0'
DEFAULT_SRV_PORT = 8099
# TODO configure drone ip addr
DRONE_IP = '0.0.0.0'
YOLO_IP = '0.0.0.0'
RSYNC_IP = '0.0.0.0'
SNIPPER_IP = '0.0.0.0'
CLASSIFIER_IP = '0.0.0.0'
SNIPPER_USER = 'benlimpa'

# Client Defaults
DEFAULT_VSN_USER = 'benlimpa'
DEFAULT_VSN_PORT = DEFAULT_SRV_PORT
DEFAULT_SSH_PORT = 22
DEFAULT_REMOTE_DIR = DEFAULT_DATA_DIR
DEFAULT_VSN_IP = DEFAULT_SRV_IP
DEFAULT_VSN_SRV = DEFAULT_SRV_IP + ':' + str(DEFAULT_SRV_PORT)
DEFAULT_THREADS = 1
DEFAULT_AUCTION_TIMEOUT = 0.3
MAX_THREADS = 20

# Yolo Defaults
DEFAULT_YOLO_PB = 'localizer/built_graph/yolo-auvsi.pb'
DEFAULT_YOLO_META = 'localizer/built_graph/yolo-auvsi.meta'
DEFAULT_YOLO_THRESH = 0.0012

processes = process_manager.ProcessManager()
c_workers = []
s_worker = None

verbose = False


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()
    # Ask the workers to join us in death
    if s_worker is not None:
        s_worker.join()
    for worker in c_workers:
        worker.join()
    sys.exit(0)


# Server #######################################################################

socketio_app = Flask(__name__)
socketio_app.config['SECRET_KEY'] = SECRET_KEY
vision_socketio_server = flask_socketio.SocketIO(socketio_app, logger=False)
server_task_queue = queue.Queue()
connected_clients = {'rsync': [], 'yolo': [], 'snipper': []}
active_auctions = {}
taken_auctions = {}
img_count = 0
hasher = hashlib.blake2b()
server_img_manager = None


class ServerWorker(threading.Thread):
    def __init__(self, in_q):  # accept args anyway even if not used
        super(ServerWorker, self).__init__()
        self.in_q = in_q  # input queue (queue.Queue)
        self.stop_req = threading.Event()  # listen for a stop request

    def run(self):
        while not self.stop_req.isSet():  # Exit run if stop was requested
            try:
                # Try to get an item from the queue
                # blocking: true; timeout: 0.05
                task = self.in_q.get(True, 0.05)

                ############ Task Format #############
                # task = {
                #     'type': 'timeout',
                #     'auction_id': str,
                #     'time_began': float
                # }
                # task = {
                #     'type': 'auction',
                #     'event_name': str,
                #     'args': {}
                # }

                # check on the progress of an auction
                if task['type'] == 'timeout':
                    if (time.time() -
                            task['time_began']) >= DEFAULT_AUCTION_TIMEOUT:
                        auction = active_auctions[task['auction_id']]

                        # reset the timer if no bids have been made
                        if len(auction['bids']) == 0:
                            task['time_began'] = time.time()
                            self.in_q.put(task)
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
                            taken_auctions[task[
                                'auction_id']] = active_auctions.pop(
                                    task['auction_id'])
                            # check on the progress of the task TODO
                    else:
                        # check again later since not enough time passed
                        self.in_q.put(task)

                # create a new auction
                elif task['type'] == 'auction':
                    # generate a random auction id
                    auction_id = str(uuid.uuid4())

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
                    self.in_q.put({
                        'type': 'timeout',
                        'time_began': time.time(),
                        'auction_id': auction_id
                    })
            except queue.Empty:
                continue

    def join(self, timeout=None):
        self.stop_req.set()
        super().join(timeout)


@vision_socketio_server.on('connect')
def vision_socketio_server_connect():
    print("Someone connected to vision server")


# Record Clients # not needed
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
def process_image(json, attempts=1):
    if verbose:
        print('process_image request made')

    # Create a new image info file
    img_info = {'time_gen': time.time()}
    manual_id = None
    try:
        img_info['id'] = json['img_id']
        manual_id = True
    except KeyError:
        img_info['id'] = ImgManager.gen_id()
        manual_id = False

    # TODO custom data dir
    img_inc_path = os.path.join(DEFAULT_DATA_DIR, img_info['id'])

    try:
        with open(img_inc_path + '.json', 'x') as f:
            json_module.dump(img_info, f)
    except FileExistsError as err:
        if manual_id:
            print('Manually Selected ID already exists: "{}"'.format(
                img_info['id']))
            return
        if attempts <= 0:
            print(
                'Process Image Error: Double image id collision for "{}"\n What are the odds? You should by a lottery ticket.'.
                format(img_info['id']))
            return
        # try again with a random image
        process_image(json, attempts - 1)
        return
    except OSError:
        # TODO handle OSError
        pass

    # TODO keep track of how many times rsync failed
    print("Telling rsync client to download image")
    server_task_queue.put({
        'type': 'auction',
        'event_name': 'rsync',
        'args': {
            'prev': {
                'event_name': 'process_image',
                'json': json
            },
            'next': [{
                'event_name': 'yolo',
                'json': {
                    'img_id': img_info['id']
                }
            }],
            'user': DRONE_USER,
            'addr': DRONE_IP,
            'img_remote_src': json['file_path'],
            'img_local_dest': img_inc_path + '.jpg'
        }
    })
    global img_count
    img_count += 1

# Intermediate step
@vision_socketio_server.on('download_complete')
def call_next(json):
    next_tasks = json['next']
    global verbose
    if verbose:
        print("Download Complete; Next Up: " + str([next_task['event_name'] for next_task in next_tasks]))
    for next_task in next_tasks:
        server_task_queue.put({
            'type': 'auction',
            'event_name': next_task['event_name'],
            'args': next_task['json']
        })


# Step 2 - find the targets in the image (yolo)


# Step 3 - cut out those targets
@vision_socketio_server.on('yolo_done')
def snip_img(json):
    global verbose
    if verbose:
        print('Yolo finished; running snipper')
    server_task_queue.put({
        'type': 'auction',
        'event_name': 'snip',
        'args': {
            'img_id': json['img_id'],
            'yolo_results': json['results']
        }
    })


# Step 4 - run shape classification on each target
#      ... do letter classification and color recognition
@vision_socketio_server.on('snipped')
def download_snipped(json):
    img_id = json['img_id']
    download_dir = json['download_dir']
    print("Telling rsync client to download snipped image")
    # WARNING do not combine these using a for loop, otherwise duplicate next tasks will be sent
    server_task_queue.put({
        'type': 'auction',
        'event_name': 'rsync',
        'args': {
            'prev': {
                'event_name': 'snipped',
                'json': json
            },
            'next': [
                {
                    'event_name': 'classify_shape',
                    'json': {
                        'img_id': json['img_id']
                    }
                },
                {
                    'event_name': 'classify_letter',
                    'json': {
                        'img_id': json['img_id']
                    }
                }
            ],
            'user': SNIPPER_USER,
            'addr': SNIPPER_IP,
            'img_remote_src': os.path.join(download_dir, img_id + '.jpg'),
            'img_local_dest': os.path.join(DEFAULT_DATA_DIR, img_id + '.jpg')
        }
    })
    server_task_queue.put({
        'type': 'auction',
        'event_name': 'rsync',
        'args': {
            'prev': {
                'event_name': 'snipped',
                'json': json
            },
            'next': [],
            'user': SNIPPER_USER,
            'addr': SNIPPER_IP,
            'img_remote_src': os.path.join(download_dir, img_id + '.json'),
            'img_local_dest': os.path.join(DEFAULT_DATA_DIR, img_id + '.json')
        }
    })


#@vision_socketio_server.on('classify')
#def classify(json):
#    server_task_queue.put({
#        'type': 'auction',
#        'event_name': 'classify_shape',
#        'args': {
#            'img_id': json['img_id']
#        }
#    })
#    server_task_queue.put({
#        'type': 'auction',
#        'event_name': 'classify_letter',
#        'args': {
#            'img_id': json['img_id']
#        }
#    })


@vision_socketio_server.on('classified')
def record_class(json):
    class_type = json['type']
    img_id = json['img_id']
    server_img_manager.set_prop(img_id, class_type, json[class_type])
    for check_class in ('shape', 'letter'):
        if server_img_manager.get_prop(img_id, check_class) is None:
            return
    # if it gets to this point without returning, then everything is done
    vision_socketio_server.emit('image_processed', {'img_id': img_id})


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
    global server_img_manager
    # TODO Server should not be using a client img_manager
    server_img_manager = ImgManager(args.data_dir, master=True)
    global img_count
    img_count = len(os.listdir(args.data_dir))
    global s_worker
    s_worker = ServerWorker(server_task_queue)
    s_worker.start()
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    vision_socketio_server.run(socketio_app, '0.0.0.0', port=int(args.port), log_output=False)


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
    # Connect to vision server
    print('Attempting to connect to server @ ' + args.vsn_addr +
          str(args.vsn_port))
    global vision_client
    vision_client = socketIO_client.SocketIO(args.vsn_addr, port=args.vsn_port)
    print('Connected to server!')

    # initialize worker and listen for tasks
    global client_id
    client_id = str(uuid.uuid4())
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
        super(ClientWorker, self).__init__()
        self.in_q = in_q  # input queue (queue.Queue)
        self.stop_req = threading.Event()  # listen for a stop request
        self.vsn_user = args.vsn_user
        self.vsn_addr = args.vsn_addr
        self.vsn_port = args.vsn_port
        self.ssh_port = args.ssh_port
        self.data_dir = args.data_dir

        self.manager = ImgManager(
            args.data_dir, {
                'user': args.vsn_user,
                'addr': args.vsn_addr,
                'port': args.ssh_port,
                'remote_dir': args.remote_dir,
                'os_type': os.name
            })

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
#                if task['type'] == 'download':
#                    processes.spawn_process_wait_for_code(
#                        'rsync -vz --progress -e "ssh -p 22" "' + self.vsn_user
#                        + '@' + self.vsn_addr + ':' + task['remote_dir'] + '')
#                else:
#                    self._do_work(task)
            except queue.Empty:
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
        global verbose
        if verbose:
            print('Called rsync with args: <' + '> <'.join(map(str, task)) +
                  '>')

        if 0 == processes.spawn_process_wait_for_code(
                'rsync -vz --progress -e "ssh -p 22" "' + task_args['user'] +
                '@' + task_args['addr'] + ':' + task_args['img_remote_src'] +
                '" ' + task_args['img_local_dest']):
            vision_client.emit(
                'download_complete', {
                    'saved_path': task_args['img_local_dest'],
                    'next': task_args['next']
                })
        else:
            vision_client.emit(
                'download_failed', {
                    'attempted_path': task_args['img_local_dest'],
                    'prev': task_args['prev']
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
            "pbLoad": args.yolo_pb,
            "metaLoad": args.yolo_meta,
            "threshold": args.yolo_threshold
        }
        self.tfnet = TFNet(yolo_options)

    # task format: [{'file_path': str}]
    def _do_work(self, task):
        img_id = task[0]['img_id']
        global verbose
        if verbose:
            print('Called yolo with args: <' + '> <'.join(map(str, task)) +
                  '>')

        img = self.manager.get_img(img_id)
        results = self.tfnet.return_predict(img)

        for result in results:
            result['confidence'] = float(result['confidence'])

        if verbose:
            print('yolo_results: ' + str(results))

        # TODO Calculate the coordinates
        #        for result in results:
        #            result
        vision_client.emit('yolo_done', {'img_id': img_id, 'results': results})

        # Result format: [{'label': str, 'confidence': int,
        #                  'topleft': {'x': int, 'y': int},
        #              'bottomright': {'x': int, 'y': int},
        #                   'coords': {'lat': float, 'lng': float}}]

    def get_event_name(self):
        return 'yolo'


class MockYoloWorker(ClientWorker):
    def __init__(self, in_q, args):
        super().__init__(in_q, args)

    def get_event_name(self):
        return 'yolo'

    def _do_work(self, task):
        img_id = task[0]['img_id']
        img = self.manager.get_img(img_id)
        results = [{
            'label': 'rectangle',
            'confidence': 5,
            'topleft': {
                'x': img.shape[1] / 2 - 10,
                'y': img.shape[0] / 2 - 10
            },
            'bottomright': {
                'x': img.shape[1] / 2 + 10,
                'y': img.shape[0] / 2 + 10
            }
        }]
        vision_client.emit('yolo_done', {'img_id': img_id, 'results': results})


def yolo_worker(args):
    if args.mock:
        client_worker(args, MockYoloWorker)
    else:
        client_worker(args, YoloWorker)


# Target Localization ##########################################################
class SnipperWorker(ClientWorker):

    # task format:
    #   [{
    #       'src_img_id': str,
    #       'loc_info': {
    #           'lat': float,
    #           'lng': float,
    #           'alt': float
    #       },
    #       'yolo_results': [],
    #   }]
    def _do_work(self, task):
        src_img_id = task[0]['img_id']
        yolo_results = task[0]['yolo_results']

        src_img = self.manager.get_img(src_img_id)

        for result in yolo_results:
            xmin = result['topleft']['x']
            xmax = result['bottomright']['x']
            ymin = result['topleft']['y']
            ymax = result['bottomright']['y']

            # Crop the image
            # TODO calculate location
            cropped_img = src_img[ymin:ymax, xmin:xmax]
            img_id = self.manager.create_new_img(
                cropped_img,
                other={
                    'parent_img_id': src_img_id,
                    'location': {
                        'lat': None,
                        'lng': None
                    }
                })
            if verbose:
                print('Snipped {} -> {}'.format(src_img_id, img_id))
            vision_client.emit('snipped', {
                'img_id': img_id,
                'download_dir': self.data_dir
            })

    def get_event_name(self):
        return 'snip'


def snipper_worker(args):
    client_worker(args, SnipperWorker)


# Classifiers ##################################################################
def classifier_worker(args):
    if args.classifier_type == 'shape':
        shape_classifier_worker(args)
    elif args.classifier_type == 'letter':
        letter_classifier_worker(args)


# Shape Classification #########################################################
class ShapeClassifierWorker(ClientWorker):
    def __init__(self, in_q, args):
        super().__init__(in_q, args)
        self.model = vision_classifier.load_model(args.model_path)
        self.data_dir = args.data_dir

        # Keras w/ Tensorflow backend bug workaround
        # This is required when using keras with tensorflow on multiple threads
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()


    # task format:
    #   [{
    #       'img_id': str,
    #   }]
    def _do_work(self, task):
        img_id = task[0]['img_id']
        img = vision_classifier.shape_img(
            os.path.join(self.data_dir, img_id + '.jpg'))

        # Keras w/ Tensorflow backend bug workaround
        with self.graph.as_default():
            prediction = vision_classifier.predict_shape(self.model, img)
            if verbose:
                print('Classified {} as a {}'.format(img_id, prediction))
            vision_client.emit('classified', {
                'img_id': img_id,
                'type': 'shape',
                'shape': prediction
            })

    def get_event_name(self):
        return 'classify_shape'


def shape_classifier_worker(args):
    client_worker(args, ShapeClassifierWorker)


# Letter Classification ########################################################
class LetterClassifierWorker(ClientWorker):
    def __init__(self, in_q, args):
        super().__init__(in_q, args)
        self.model = vision_classifier.load_model(args.model_path)
        self.data_dir = args.data_dir

        # Keras w/ Tensorflow backend bug workaround
        # This is required when using keras with tensorflow on multiple threads
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()

    # task format:
    #   [{
    #       'img_id': str,
    #   }]
    def _do_work(self, task):
        img_id = task[0]['img_id']
        img = vision_classifier.letter_img(
            os.path.join(self.data_dir, img_id + '.jpg'))

        # Keras w/ Tensorflow backend bug workaround
        with self.graph.as_default():
            prediction = vision_classifier.predict_letter(self.model, img)
            if verbose:
                print('Classified {} as a {}'.format(img_id, prediction))
            vision_client.emit('classified', {
                'img_id': img_id,
                'type': 'letter',
                'letter': prediction
            })

    def get_event_name(self):
        return 'classify_letter'


def letter_classifier_worker(args):
    client_worker(args, LetterClassifierWorker)


# Parse command line arguments #################################################
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument(
        '--data-dir',
        action='store',
        dest='data_dir',
        default=DEFAULT_DATA_DIR,
        help=
        'specify the working directory for images and their associated metadata'
    )

    subparsers = parser.add_subparsers(help='sub-command help')

    # Server Parser ####################################################
    server_port = None
    server_parser = subparsers.add_parser(
        'server', help='start the primary vision server')
    server_parser.add_argument(
        '-p'
        '--port',
        action='store',
        dest='port',
        default=DEFAULT_SRV_PORT,
        help='specify server port')
    server_parser.add_argument(
        '--drone-user',
        action='store',
        dest='drone_user',
        default=DRONE_USER,
        help='specify username for drone companion computer')
    server_parser.set_defaults(func=server_worker)

    # Client Parsers ###################################################
    client_parser = subparsers.add_parser(
        'client', help='start a worker client')
    client_parser.add_argument(
        "--vsn-addr",
        action='store',
        dest='vsn_addr',
        default=DEFAULT_VSN_IP,
        help='ip address of the primary vision server')
    client_parser.add_argument(
        "--vsn-port",
        action='store',
        dest='vsn_port',
        default=DEFAULT_VSN_PORT,
        help='SocketIO port of the primary vision server')
    client_parser.add_argument(
        "--ssh-port",
        action='store',
        dest='ssh_port',
        default=DEFAULT_SSH_PORT,
        help='SocketIO port of the primary vision server')
    client_parser.add_argument(
        "--remote-data-dir",
        action='store',
        dest='remote_dir',
        default=DEFAULT_REMOTE_DIR,
        help='data directory of the primary vision server')
    client_parser.add_argument(
        '--user',
        action='store',
        dest='vsn_user',
        default=DEFAULT_VSN_USER,
        help='ssh user for rsync to the primary vision server')
    client_parser.add_argument(
        "--threads",
        action='store',
        dest='threads',
        default=DEFAULT_THREADS,
        choices=range(1, MAX_THREADS + 1),
        help='number of threads to run the client worker with')
    client_parser.add_argument(
        '--mock',
        action='store_true',
        dest='mock',
        default=False,
        help='use the mock version of the client instead')

    client_subparsers = client_parser.add_subparsers()

    # Rsync specific args
    rsync_parser = client_subparsers.add_parser(
        'rsync', help='start an rsync worker client')
    rsync_parser.set_defaults(func=rsync_worker)

    # Yolo specific args
    yolo_parser = client_subparsers.add_parser(
        'yolo', help='start a YOLO worker client')
    yolo_parser.add_argument(
        "--protobuf",
        action='store',
        dest='yolo_pb',
        default=DEFAULT_YOLO_PB,
        help='specify the protobuf file of the built graph to load')
    yolo_parser.add_argument(
        "--meta",
        action='store',
        dest='yolo_meta',
        default=DEFAULT_YOLO_META,
        help='specify the meta file of the built graph to load')
    yolo_parser.add_argument(
        "--thresh",
        action='store',
        dest='yolo_threshold',
        default=DEFAULT_YOLO_THRESH,
        help='specify the threshold for recognizing a target')
    yolo_parser.set_defaults(func=yolo_worker)

    # snipper specific args
    snipper_parser = client_subparsers.add_parser(
        'snipper', help='start a snipper worker client')
    snipper_parser.set_defaults(func=snipper_worker)

    # classifier specific args
    classifier_parser = client_subparsers.add_parser(
        'classifier', help='start a classifier worker client')
    classifier_parser.add_argument(
        'classifier_type',
        choices=('shape', 'letter'),
        help='specify which type of classifier')
    classifier_parser.add_argument(
        '--model',
        action='store',
        dest=
        'model_path',  # cannot specify default since there are two different types
        help='specify the classification model to load')
    classifier_parser.set_defaults(func=classifier_worker)

    args = parser.parse_args()
    #global verbose
    verbose = args.verbose
    args.func(args)