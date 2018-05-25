import signal
import socketIO_client
from argparse
import threading
import time

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

import vision

MAX_TIMEOUT = 5  # seconds
MOCK_RECEIVERS = ['rsync', 'yolo', 'snip', 'classify_shape', 'classify_letter']
DEFAULT_PATH = '~/Projects/vision/target_generator/DATA/all_imgs/0000232.jpg'

class TestVisionServer(unittest.TestCase):
    def setUp(self):
        # create a list to save the collected socketio messages
        self.messages = []

        # start the server worker
        args = argparse.Namespace()
        args.port = 8099
        self.running_threads = []
        self.running_threads.append(
            threading.Thread(target=vision.server_worker, args=(args)))

        # connect a dummy client
        self.client = socketIO_client.SocketIO('0.0.0.0', 8099)
        self.client.on('rsync', self.handle_rsync)
        self.client.on('yolo', self.handle_yolo)
        self.client.on('snip', self.handle_snip)
        print('connected')
        pass

    def tearDown(self):
        pass

    def handle_message(self, *args):
        self.messages.append(args)

    def handle_rsync(*args):
        self.messages.append({'name': 'rsync', 'args': args})

    def handle_yolo(*args):
        self.messages.append({'name': 'yolo', 'args': args})

    def handle_snip(*args):
        self.messages.append({'name': 'snip', 'args': args})

    def timeout_call(self):
        pass

    def test_process_img(self):
        start_mess_count = len(self.messages)
        start_time = time.ctime()
        dummy_path = 'dummy_path/dummy_subfolder/dummy_file.jpg'
        self.client.emit(
            'process_image',
            {'file_path': dummy_path})
        while (len(self.messages) == org_mess_count
               ) and (time.ctime() - start_time) >= MAX_TIMEOUT:
            time.sleep(0.05)
        self.assertEqual(
            self.messages[len(self.messages)], {
                'name':
                'rsync',
                'args': [{
                    'prev': None,
                    'next': None,
                    'user': 'pi',
                    'addr': 'INSERT_DRONE_IP',
                    'img_remote_src': dummy_path,
                    'img_local_printf '\033]2;%s\033\\' 'server1'dest': '/home/benlimpa/')
                }]
            })

def make_listener(name):
    def listener(*args):
        print('{}: {}'.format(name, args))
    return listener

def create_mock_listeners():
    socket = socketIO.SocketIO('0.0.0.0', 8099)
    for name in MOCK_RECEIVERS:
        socket.on(name, make_listener(name))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('type')
    parser.add_argument('-t', dest='target', default=DEFAULT_PATH)
    args = parser.parse_args()

    if args.type == 'main':
        unittest.main()
    elif args.type == 'listen':
        create_mock_listeners()
    elif args.type == 'send':
        with socketIO_client.SocketIO('0.0.0.0', 8099) as socket:
            socket.emit('process_image', {'file_path': args.target})
