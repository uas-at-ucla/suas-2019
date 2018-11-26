import os
import sys
import signal
import argparse
import socketIO_client
import time
import subprocess
import re

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager  # noqa: E402

# Multithreading
import threading  # noqa: E402  # multithreading library
import queue  # noqa: E402  # input/output queues

# Classification dependencies
sys.path.insert(0, './classifier')
# Client dependency
sys.path.insert(0, './util')
from img_manager import ImgManager  # noqa: E402
import coordinates

from config import Config

# Worker dependencies
sys.path.insert(0, './workers')
from rsync_worker import RsyncWorker
from yolo_worker import YoloWorker, MockYoloWorker
from snipper_worker import SnipperWorker
from classifier_workers import ShapeClassifierWorker, LetterClassifierWorker

from clients import VisionClient
from server import VisionServer

# Defaults ####################################################################

print('cwd: ' + os.getcwd())
print('dir: ' + Config.DOCKER_DATA_DIR.value)

processes = process_manager.ProcessManager()
s_workers = [] # to make the server worker accessible by server.py
c_workers = []
img_query_worker = None
img_query_worker_stop = threading.Event()

verbose = False

def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()

    # Ask the workers to join us in death
    for worker in s_workers:
        worker.join()
    if img_query_worker is not None:
        img_query_worker_stop.set()
        img_query_worker.join()
    for worker in c_workers:
        worker.join()
    sys.exit(0)


# Server ######################################################################

def query_for_imgs(stop, drone_user, drone_ip, folder, server_port,
                   interval=10, remote_encoding='utf-8'):
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


def setup_server(args):
    # setup worker to query the drone for images
    img_query_worker = threading.Thread(
        target=query_for_imgs,
        args=(img_query_worker_stop, args.drone_user, Config.DRONE_IP.value,
              Config.DRONE_IMG_FOLDER.value, args.port))
    img_query_worker.start()
    VisionServer(args=args, server_workers=s_workers, verbose=verbose)


# Clients #####################################################################

def setup_client(worker_class):
    VisionClient(args=args, worker_class=worker_class, client_workers=c_workers,
                 processes=processes, verbose=verbose)


# Rsync file synchronization
def rsync_worker(args):
    setup_client(RsyncWorker)


# YOLO image classification
def yolo_worker(args):
    if args.mock:
        setup_client(MockYoloWorker)
    else:
        setup_client(YoloWorker)


# Target Localization
def snipper_worker(args):
    setup_client(SnipperWorker)


# Classifiers
def classifier_worker(args):
    if args.classifier_type == 'shape':
        setup_client(ShapeClassifierWorker)
    elif args.classifier_type == 'letter':
        setup_client(LetterClassifierWorker)


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
    server_parser.set_defaults(func=setup_server)

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
