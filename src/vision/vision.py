import os
import sys
import signal
import argparse
from flask import Flask
import flask_socketio
import socketIO_client
import logging

import time
import uuid
import json as json_module
import hashlib
import subprocess
import re

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager  # noqa: E402

# Yolo dependencies
from darkflow.net.build import TFNet  # noqa: E402  # yolo neural net

# Classification dependencies
sys.path.insert(0, './classifier')
from classifier import vision_classifier  # noqa: E402
import tensorflow as tf  # noqa: E402

# Multithreading
import threading  # noqa: E402  # multithreading library
import queue  # noqa: E402  # input/output queues

# Snipper dependencies
# cv2 imported above # cropping and saving images

# Client dependency
sys.path.insert(0, './util')
from util.img_manager import ImgManager  # noqa: E402
import coordinates

# Module dependencies
from config import Config
sys.path.insert(0, './workers')
from server_worker import PriorityItem, ServerWorker


# Defaults ####################################################################

print('cwd: ' + os.getcwd())
print('dir: ' + Config.DOCKER_DATA_DIR.value)

processes = process_manager.ProcessManager()
c_workers = []
s_worker = None
img_query_worker = None
img_query_worker_stop = threading.Event()

verbose = False


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()

    # Ask the workers to join us in death
    if s_worker is not None:
        s_worker.join()
    if img_query_worker is not None:
        img_query_worker_stop.set()
        img_query_worker.join()
    for worker in c_workers:
        worker.join()
    sys.exit(0)


# Server ######################################################################

# - Critical database updates preempt everything. (priority = -1)
# - Requests from the front end preempt almost all tasks. (priority = 0)
# - Archive database updates preempt auctions. (priority = 1)
# - Auctions and timeouts are lowest priority. (priority = 2)

socketio_app = Flask(__name__)
socketio_app.config['SECRET_KEY'] = Config.SECRET_KEY.value
vision_socketio_server = flask_socketio.SocketIO(socketio_app, logger=False)
server_task_queue = queue.PriorityQueue()
connected_clients = {'rsync': [], 'yolo': [], 'snipper': []}
img_count = 0
hasher = hashlib.blake2b()
server_img_manager = None


@vision_socketio_server.on('connect')
def vision_socketio_server_connect():
    print("Someone connected to vision server")


@vision_socketio_server.on('bid')
def receive_bid(json):
    s_worker.active_auctions[json['auction_id']]['bids'].append(json['bid'])


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
    img_inc_path = os.path.join(Config.DOCKER_DATA_DIR.value, img_info['id'])

    try:
        with open(img_inc_path + '.json', 'x') as f:
            json_module.dump(img_info, f)
    except FileExistsError:
        if manual_id:
            print('Manually Selected ID already exists: "{}"'.format(
                img_info['id']))
            return
        if attempts <= 0:
            print('Process Image Error: Double image id collision for "{}"\n \
                What are the odds? You should by a lottery ticket.'.format(
                img_info['id']))
            return
        # try again with a random image
        process_image(json, attempts - 1)
        return
    except OSError:
        # TODO handle OSError
        pass

    # TODO keep track of how many times rsync failed
    print("Telling rsync client to download image")
    # yapf: disable
    server_task_queue.put(PriorityItem(2, {
        'type': 'auction',
        'event_name': 'rsync',
        'args': {
            'prev': {
                'event_name': 'process_image',
                'json': json
            },
            'next': {
                'func': 'syncronize_img_info',
                'args': (
                    img_info['id'],
                    img_inc_path + '-drone.json'
                )
            },
            'ssh_id_path': Config.DRONE_SSH_ID.value,
            'hosts_path': Config.DRONE_HOST_KEY.value,
            'user': Config.DRONE_USER.value,
            'addr': Config.DRONE_IP.value,
            'img_remote_src': [json['file_path'], json['info_path']],
            'img_local_dest': [img_inc_path + '.jpg',
                               img_inc_path + '-drone.json']
        }
    }))
    # yapf: enable
    server_task_queue.put(
        PriorityItem(
            1, {
                'type':
                'add_record',
                'sql_statement':
                "insert into Images values ('{}','{}')".format(
                    img_info['id'], 'raw')
            }))
    vision_socketio_server.emit('new_raw', {'img_id': img_info['id']})
    global img_count
    img_count += 1


def syncronize_img_info(img_id, new_info_file):
    data = None
    try:
        with open(new_info_file, 'r') as f:
            data = json_module.load(f)
    except OSError:
        pass  # fail silently
    server_img_manager.set_prop(img_id, 'time_gen', data['time'])
    server_img_manager.set_prop(img_id, 'lat', data['latitude'])
    server_img_manager.set_prop(img_id, 'lng', data['longitude'])
    server_img_manager.set_prop(img_id, 'heading', data['heading'])
    server_img_manager.set_prop(img_id, 'altitude', data['altitude'])
    img = server_img_manager.get_img(img_id)
    server_img_manager.set_prop(img_id, 'width_px', img.shape[2])
    server_img_manager.set_prop(img_id, 'height_px', img.shape[1])
    server_task_queue.put(
        PriorityItem(2, {
            'type': 'auction',
            'event_name': 'yolo',
            'args': {
                'img_id': img_id
            }
        }))


# Intermediate step
@vision_socketio_server.on('download_complete')
def call_next(json):
    globals()[json['next']['func']](*json['next']['args'])


def do_auction(*auctions):
    for auction in auctions:
        server_task_queue.put(
            PriorityItem(
                2, {
                    'type': 'auction',
                    'event_name': auction['event_name'],
                    'args': auction['json']
                }))


# Step 2 - find the targets in the image (yolo)


# Step 3 - cut out those targets
@vision_socketio_server.on('yolo_done')
def snip_img(json):
    global verbose
    if verbose:
        print('Yolo finished; running snipper')
    server_task_queue.put(
        PriorityItem(
            2, {
                'type': 'filter_and_snip',
                'img_id': json['img_id'],
                'yolo_results': json['results']
            }))


# Step 4 - run shape classification on each target
#      ... do letter classification and color recognition
@vision_socketio_server.on('snipped')
def download_snipped(json):
    img_id = json['img_id']
    download_dir = json['download_dir']
    print("Telling rsync client to download snipped image")
    # yapf: disable
    # WARNING: this auctions must occur as one event
    # to prevent duplicate "next" calls
    server_task_queue.put(PriorityItem(2, {
        'type': 'auction',
        'event_name': 'rsync',
        'args': {
            'prev': {
                'event_name': 'snipped',
                'json': json
            },
            'next': {
                'func': 'do_auction',
                'args': (
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
                )
            },
            'ssh_id_path': Config.SNIPPER_SSH_ID.value,
            'hosts_path': Config.SNIPPER_HOST_KEY.value,
            'user': Config.SNIPPER_USER.value,
            'addr': Config.SNIPPER_IP.value,
            'img_remote_src': [os.path.join(download_dir, img_id + ext)
                               for ext in ('.jpg', '.json')],
            'img_local_dest': [os.path.join(Config.DOCKER_DATA_DIR.value, 
                                            img_id + ext)
                               for ext in ('.jpg', '.json')]
        }
    }))
    # yapf: enable
    server_task_queue.put(
        PriorityItem(
            1, {
                'type':
                'add_record',
                'sql_statement':
                "insert into Images values ('{}', 'localized')".format(img_id)
            }))
    vision_socketio_server.emit('new_localized', json)


@vision_socketio_server.on('classified')
def record_class(json):
    class_type = json['type']
    img_id = json['img_id']
    server_img_manager.set_prop(img_id, class_type, json[class_type])
    for check_class in ('shape', 'letter'):
        if server_img_manager.get_prop(img_id, check_class) is None:
            return
    # if it gets to this point without returning, then everything is done
    server_task_queue.put(
        PriorityItem(
            1, {
                'type':
                'add_record',
                'sql_statement':
                "update Images set Type = 'classified' where ImageID='{}'".
                format(img_id)
            }))
    vision_socketio_server.emit('new_classified', {'img_id': img_id})
    vision_socketio_server.emit('image_processed', {'img_id': img_id})


@vision_socketio_server.on('manual_request')
def manual_request(json):
    json['args']['manual'] = True
    server_task_queue.put(
        PriorityItem(
            0, {
                'type': 'auction',
                'event_name': json['event_name'],
                'args': json['args']
            }))


@vision_socketio_server.on('manual_request_done')
def manual_request_done(json):
    vision_socketio_server.emit('manual_request_done', json)


@vision_socketio_server.on('get_all_images')
def return_all_images():
    server_task_queue.put(PriorityItem(0, {'type': 'retrieve_records'}))


@vision_socketio_server.on('calc_target_coords')
def call_calc_target_coords(json):
    lat, lng = coordinates.calculate_target_coordinates(
        target_pos_pixel=json['target_pixel_pos'],
        parent_img_real_coords=json['parent_img_real_coords'],
        parent_img_dimensions_pixel=json['parent_img_dimensions'],
        altitude=json['altitude'],
        heading=json['heading'])
    vision_socketio_server.emit('found_target_coords', {
        'lat': lat,
        'lng': lng
    })


def query_for_imgs(stop,
                   drone_user,
                   drone_ip,
                   folder,
                   server_port,
                   interval=10,
                   remote_encoding='utf-8'):
    client = socketIO_client.SocketIO('0.0.0.0', server_port)
    downloaded_files = set()
    while not stop.is_set():  # stop running when told to do so
        try:
            # get a listing of all the files in the folder
            # throws TimeoutExpired if timeout (interval in secs) runs out
            result = subprocess.run([
                'ssh', '-i ' + Config.DRONE_SSH_ID.value,
                '-o UserKnownHostsFile=' + Config.DRONE_HOST_KEY.value,
                drone_user + '@' + drone_ip, 'ls -1 {}'.format(folder)
            ],
                                    timeout=interval,
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL)
            # throws CalledProcessError is something went wrong with ssh
            result.check_returncode()
            # group(1) as to not include the newline
            # append a newline to the end in case the result does not terminate
            # with one. Also check extension to include only json files
            # (since those are created after the JPG)
            new_items = set(
                item.group(1)
                for item in re.finditer('(.+)\n',
                                        result.stdout.decode('utf-8') + '\n')
                if '.' in item.group(1) and item.group(1).split('.')[1] ==
                'json' and item.group(1) not in downloaded_files)
            downloaded_files |= new_items
            for item in new_items:
                client.emit(
                    'process_image', {
                        'img_path':
                        os.path.join(folder,
                                     item.split('.')[0] + '.JPG'),
                        'info_path':
                        os.path.join(folder,
                                     item.split('.')[0] + '.json')
                    })
        except subprocess.CalledProcessError:
            time.sleep(interval)  # wait the interval before trying again
        except subprocess.TimeoutExpired:
            continue  # already waited the interval with the timeout


def setup_server_worker(args):
    # setup worker to query the drone for images
    img_query_worker = threading.Thread(
        target=query_for_imgs,
        args=(img_query_worker_stop, args.drone_user, Config.DRONE_IP.value,
              Config.DRONE_IMG_FOLDER.value, args.port))
    img_query_worker.start()
    # setup the database:
    global server_img_manager
    # TODO Server should not be using a client img_manager
    server_img_manager = ImgManager(Config.DOCKER_DATA_DIR.value, master=True)
    global img_count
    img_count = len(os.listdir(Config.DOCKER_DATA_DIR.value))
    global s_worker
    s_worker = ServerWorker(server_task_queue, vision_socketio_server,
                            server_img_manager)
    s_worker.start()
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    vision_socketio_server.run(
        socketio_app, '0.0.0.0', port=int(args.port), log_output=False)


# Clients #####################################################################
client_id = None
work_queue = queue.Queue()
vision_client = None


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
    print('Attempting to connect to server @ ' + args.vsn_addr + ':' +
          str(args.vsn_port))
    global vision_client
    vision_client = socketIO_client.SocketIO(args.vsn_addr, port=args.vsn_port)
    print('Connected to server!')

    # initialize worker and listen for tasks
    global client_id
    client_id = str(uuid.uuid4())
    global work_queue
    for i in range(0, args.threads):
        c_worker = worker_class(
            in_q=work_queue, socket_client=vision_client, args=args)
        c_worker.start()
        c_workers.append(c_worker)
    vision_client.on(c_workers[0].get_event_name(), client_bid_for_task)
    vision_client.on(c_workers[0].get_event_name() + '_' + client_id,
                     client_add_task)
    vision_client.wait()


class ClientWorker(threading.Thread):
    def __init__(self, in_q, socket_client,
                 args):  # accept args anyway even if not used
        super(ClientWorker, self).__init__()
        self.in_q = in_q  # input queue (queue.Queue)
        self.stop_req = threading.Event()  # listen for a stop request
        self.vsn_user = args.vsn_user
        self.vsn_addr = args.vsn_addr
        self.vsn_port = args.vsn_port
        self.ssh_port = args.ssh_port
        self.data_dir = Config.DOCKER_DATA_DIR.value
        self.socket_client = socket_client

        self.manager = ImgManager(
            Config.DOCKER_DATA_DIR.value, {
                'user': args.vsn_user,
                'addr': args.vsn_addr,
                'port': args.ssh_port,
                'remote_dir': args.remote_dir,
                'os_type': os.name
            })

    def _emit(self, task, event_name, args):
        if 'manual' in task[0] and task[0]['manual']:
            args['event_name'] = event_name
            self.socket_client.emit('manual_request_done', args)
        else:
            self.socket_client.emit(event_name, args)

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
            except queue.Empty:
                continue

    def join(self, timeout=None):
        self.stop_req.set()
        super().join(timeout)


# Rsync file synchronization ##################################################
class RsyncWorker(ClientWorker):
    # task format: [{'prev': {}, 'next': {}, 'user': str, 'addr': str,
    #                'img_remote_src': str, 'img_local_dest': str}]
    def _do_work(self, task):
        task_args = task[0]
        global verbose
        if verbose:
            print('Called rsync with args: <' + '> <'.join(map(str, task)) +
                  '>')

        success = True
        for i in range(len(task_args['img_remote_src'])):
            remote = task_args['img_remote_src'][i]
            local = task_args['img_local_dest'][i]

            rsync_command = ('rsync -vz --progress -e "ssh -p 22 '
                             '-i {id_file} -o UserKnownHostsFile={hosts_file}"'
                             ' {user}@{ip}:{remote_path} {local_path}').format(
                                 id_file=task_args['ssh_id_path'],
                                 hosts_file=task_args['hosts_path'],
                                 user=task_args['user'],
                                 ip=task_args['addr'],
                                 remote_path=remote,
                                 local_path=local)
            if verbose:
                print('Spawning rsync: ' + rsync_command)
            if 0 != processes.spawn_process_wait_for_code(rsync_command):
                success = False
        if success:
            self._emit(
                task, 'download_complete', {
                    'saved_path': task_args['img_local_dest'],
                    'next': task_args['next']
                })
        else:
            self._emit(
                task, 'download_failed', {
                    'attempted_path': task_args['img_local_dest'],
                    'prev': task_args['prev']
                })

    def get_event_name(self):
        return 'rsync'


def rsync_worker(args):
    client_worker(args, RsyncWorker)


# YOLO image classification ###################################################
class YoloWorker(ClientWorker):
    def __init__(self, in_q, socket_client, args):
        super().__init__(in_q, socket_client, args)

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
        self._emit(task, 'yolo_done', {'img_id': img_id, 'results': results})

        # Result format: [{'label': str, 'confidence': int,
        #                  'topleft': {'x': int, 'y': int},
        #              'bottomright': {'x': int, 'y': int},
        #                   'coords': {'lat': float, 'lng': float}}]

    def get_event_name(self):
        return 'yolo'


class MockYoloWorker(ClientWorker):
    def __init__(self, in_q, socket_client, args):
        super().__init__(in_q, socket_client, args)

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
        self._emit(task, 'yolo_done', {'img_id': img_id, 'results': results})


def yolo_worker(args):
    if args.mock:
        client_worker(args, MockYoloWorker)
    else:
        client_worker(args, YoloWorker)


# Target Localization #########################################################
class SnipperWorker(ClientWorker):
    def __init__(self, in_q, socket_client, args):
        super().__init__(in_q, socket_client, args)
        self.real_data_dir = args.real_data_dir

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
        print('Snipper called')
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
                        'lat': result['coords']['lat'],
                        'lng': result['coords']['lng']
                    }
                })
            if verbose:
                print('Snipped {} -> {}'.format(src_img_id, img_id))

            self._emit(task, 'snipped', {
                'img_id': img_id,
                'download_dir': self.real_data_dir
            })

    def get_event_name(self):
        return 'snip'


def snipper_worker(args):
    client_worker(args, SnipperWorker)


# Classifiers #################################################################
def classifier_worker(args):
    if args.classifier_type == 'shape':
        shape_classifier_worker(args)
    elif args.classifier_type == 'letter':
        letter_classifier_worker(args)


# Shape Classification ########################################################
class ShapeClassifierWorker(ClientWorker):
    def __init__(self, in_q, socket_client, args):
        super().__init__(in_q, socket_client, args)
        self.model = vision_classifier.load_model(args.model_path)
        self.data_dir = Config.DOCKER_DATA_DIR.value

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
        self.manager.get_img(img_id)
        img = vision_classifier.shape_img(
            os.path.join(self.data_dir, img_id + '.jpg'))

        # Keras w/ Tensorflow backend bug workaround
        with self.graph.as_default():
            prediction = vision_classifier.predict_shape(self.model, img)
            if verbose:
                print('Classified {} as a {}'.format(img_id, prediction))
            self._emit(task, 'classified', {
                'img_id': img_id,
                'type': 'shape',
                'shape': prediction
            })

    def get_event_name(self):
        return 'classify_shape'


def shape_classifier_worker(args):
    client_worker(args, ShapeClassifierWorker)


# Letter Classification #######################################################
class LetterClassifierWorker(ClientWorker):
    def __init__(self, in_q, socket_client, args):
        super().__init__(in_q, socket_client, args)
        self.model = vision_classifier.load_model(args.model_path)
        self.data_dir = Config.DOCKER_DATA_DIR.value

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
        self.manager.get_img(img_id)
        img = vision_classifier.letter_img(
            os.path.join(self.data_dir, img_id + '.jpg'))

        # Keras w/ Tensorflow backend bug workaround
        with self.graph.as_default():
            prediction = vision_classifier.predict_letter(self.model, img)
            if verbose:
                print('Classified {} as a {}'.format(img_id, prediction))
            self._emit(task, 'classified', {
                'img_id': img_id,
                'type': 'letter',
                'letter': prediction
            })

    def get_event_name(self):
        return 'classify_letter'


def letter_classifier_worker(args):
    client_worker(args, LetterClassifierWorker)


# Parse command line arguments ################################################
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers(help='sub-command help')

    # Server Parser ####################################################
    server_port = None
    server_parser = subparsers.add_parser(
        'server', help='start the primary vision server')
    server_parser.add_argument('-v', '--verbose', action='store_true')
    server_parser.add_argument('-p'
                               '--port',
                               action='store',
                               dest='port',
                               default=Config.DEFAULT_SRV_PORT.value,
                               help='specify server port')
    server_parser.add_argument(
        '--drone-user',
        action='store',
        dest='drone_user',
        default=Config.DRONE_USER.value,
        help='specify username for drone companion computer')
    server_parser.set_defaults(func=setup_server_worker)

    # Client Parsers ###################################################
    client_parser = subparsers.add_parser(
        'client', help='start a worker client')
    client_parser.add_argument('-v', '--verbose', action='store_true')
    client_parser.add_argument(
        "--vsn-addr",
        action='store',
        dest='vsn_addr',
        default=Config.DEFAULT_VSN_IP.value,
        help='ip address of the primary vision server')
    client_parser.add_argument(
        "--vsn-port",
        action='store',
        dest='vsn_port',
        default=Config.DEFAULT_VSN_PORT.value,
        help='SocketIO port of the primary vision server')
    client_parser.add_argument(
        "--ssh-port",
        action='store',
        dest='ssh_port',
        default=Config.DEFAULT_SSH_PORT.value,
        help='SocketIO port of the primary vision server')
    client_parser.add_argument(
        "--remote-data-dir",
        action='store',
        dest='remote_dir',
        default=Config.DEFAULT_REMOTE_DIR.value,
        help='data directory of the primary vision server')
    client_parser.add_argument(
        '--user',
        action='store',
        dest='vsn_user',
        default=Config.DEFAULT_VSN_USER.value,
        help='ssh user for rsync to the primary vision server')
    client_parser.add_argument(
        "--threads",
        action='store',
        dest='threads',
        default=Config.DEFAULT_THREADS.value,
        choices=range(1, Config.MAX_THREADS.value + 1),
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
        default=Config.DEFAULT_YOLO_PB.value,
        help='specify the protobuf file of the built graph to load')
    yolo_parser.add_argument(
        "--meta",
        action='store',
        dest='yolo_meta',
        default=Config.DEFAULT_YOLO_META.value,
        help='specify the meta file of the built graph to load')
    yolo_parser.add_argument(
        "--thresh",
        action='store',
        dest='yolo_threshold',
        default=Config.DEFAULT_YOLO_THRESH.value,
        help='specify the threshold for recognizing a target')
    yolo_parser.set_defaults(func=yolo_worker)

    # snipper specific args
    snipper_parser = client_subparsers.add_parser(
        'snipper', help='start a snipper worker client')
    snipper_parser.add_argument(
        "--real-dir",
        action='store',
        dest='real_data_dir',
        required=True,
        help='specify the directory the container volume is mounted to')
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
        # cannot specify default since there are two different types
        dest='model_path',
        help='specify the classification model to load')
    classifier_parser.set_defaults(func=classifier_worker)

    args = parser.parse_args()
    verbose = args.verbose
    if verbose:
        print("Running in verbose mode.")
    args.func(args)
