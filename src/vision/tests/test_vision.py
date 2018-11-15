import signal
import json
import socketIO_client
import argparse
import threading
import time
import os
import sys
import unittest
import xml.etree.ElementTree

os.chdir(os.path.dirname(os.path.realpath(__file__)))
os.chdir('..')
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager

sys.path.insert(0, '..')
import vision

MAX_TIMEOUT = 5  # seconds
MOCK_RECEIVERS = ['rsync', 'yolo', 'snip', 'classify_shape', 'classify_letter']
DEFAULT_IMG_PATH = os.path.dirname(__file__) + '/../data_local/sample_img.jpg'
DEFAULT_INFO_PATH = os.path.dirname(
    __file__) + '/../data_local/sample_drone_info.json'


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

    def handle_rsync(self, *args):
        self.messages.append({'name': 'rsync', 'args': args})

    def handle_yolo(self, *args):
        self.messages.append({'name': 'yolo', 'args': args})

    def handle_snip(self, *args):
        self.messages.append({'name': 'snip', 'args': args})

    def timeout_call(self):
        pass

    def test_process_img(self):
        start_mess_count = len(self.messages)
        start_time = time.ctime()
        dummy_path = 'dummy_path/dummy_subfolder/dummy_file.jpg'
        self.client.emit('process_image', {'file_path': dummy_path})
        while (len(self.messages) == start_mess_count
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
                    'img_local_dest': dummy_path
                }]
            })


def make_listener(name):
    def listener(*args):
        print('{}: {}'.format(name, args))

    return listener


letter_correct = 0
shape_correct = 0
total = 0
total_size = 0
done_emitting = False


def create_mock_listeners():
    socket = socketIO_client.SocketIO('0.0.0.0', 8099)
    for name in MOCK_RECEIVERS:
        socket.on(name, make_listener(name))


def sample_manual_request(args):
    def printmess(*json):
        print(json)

    socket = socketIO_client.SocketIO('0.0.0.0', 8099)
    socket.on('manual_request_done', printmess)
    socket.emit(
        'manual_request', {
            'event_name': 'snip',
            'args': {
                'img_id':
                args.target,
                'yolo_results': [{
                    'topleft': {
                        'x': 25,
                        'y': 25
                    },
                    'bottomright': {
                        'x': 50,
                        'y': 50
                    }
                }]
            }
        })
    socket.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('type')
    parser.add_argument('-t', dest='target', default=DEFAULT_IMG_PATH)
    parser.add_argument('-i', dest='info', default=DEFAULT_INFO_PATH)
    args = parser.parse_args()

    if args.type == 'main':
        unittest.main()
    elif args.type == 'listen':
        create_mock_listeners()
    elif args.type == 'send':
        with socketIO_client.SocketIO('0.0.0.0', 8099) as socket:
            socket.emit('process_image', {
                'file_path': args.target,
                'info_path': args.info
            })
    elif args.type == 'evaluate':
        with socketIO_client.SocketIO('0.0.0.0', 8099) as socket:

            def check_img(*js):
                img_id = js[0]['img_id']
                with open(os.path.join('./data_local', img_id + '.json')) as f:
                    data = json.load(f)
                    root = xml.etree.ElementTree.parse(
                        os.path.join(
                            args.target,
                            data['parent_img_id'] + '.xml')).getroot()
                    real_shape = root[4][0].text
                    real_letter = root[5].text

                    global total
                    global letter_correct
                    global shape_correct
                    total += 1
                    if data['letter'] == real_letter:
                        letter_correct += 1
                    if data['shape'] == real_shape:
                        shape_correct += 1

                print('Total: {}    Letters: {}     Shapes: {}'.format(
                    total, letter_correct, shape_correct))
                if done_emitting and total == total_size:
                    pass

            socket.on('image_processed', check_img)
            for img_path in os.listdir(args.target):
                img_split = img_path.split('.')
                if img_split[1] == 'jpg':
                    socket.emit(
                        'process_image', {
                            'file_path': os.path.join(args.target, img_path),
                            'img_id': img_split[0]
                        })
            socket.wait()
    elif args.type == 'manual':
        sample_manual_request(args)
