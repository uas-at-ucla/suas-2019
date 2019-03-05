import os
from flask import Flask
import flask_socketio
import logging
import time
import json as json_module
import hashlib
import queue

from img_manager import ImgManager
from config import Config
from server_worker import PriorityItem, ServerWorker

# - Critical database updates preempt everything. (priority = -1)
# - Requests from the front end preempt almost all tasks. (priority = 0)
# - Archive database updates preempt auctions. (priority = 1)
# - Auctions and timeouts are lowest priority. (priority = 2)

class VisionServer:
    def __init__(self, args, verbose):
        self.socketio_app = Flask(__name__)
        self.socketio_app.config['SECRET_KEY'] = Config.SECRET_KEY.value
        self.socketio_server = flask_socketio.SocketIO(self.socketio_app, 
                                                       logger=False)
        self.task_queue = queue.PriorityQueue()
        self.server_workers = []
        self.connected_clients = {'rsync': [], 'yolo': [], 'snipper': []}
        self.img_count = len(os.listdir(Config.DOCKER_DATA_DIR.value))
        self.hasher = hashlib.blake2b()
        # setup the database:
        self.img_manager = ImgManager(Config.DOCKER_DATA_DIR.value, master=True)
        # TODO Server should not be using a client img_manager
        self.s_worker = ServerWorker(self.task_queue, self.socketio_server,
                                     self.img_manager)

        self.funcs = {
            'do_auction': self.do_auction,
            'syncronize_img_info': self.syncronize_img_info
        }
        self.verbose = verbose
        self.attach_listeners()

        self.s_worker.start()
        self.server_workers.append(self.s_worker)
        logging.getLogger('werkzeug').setLevel(logging.ERROR)
        self.socketio_server.run(
            self.socketio_app, '0.0.0.0', port=int(args.port), log_output=False)


    # Non-listener functions

    def kill_workers(self):
        self.s_worker.join()

    def syncronize_img_info(self, img_id, new_info_file):
        data = None
        try:
            with open(new_info_file, 'r') as f:
                data = json_module.load(f)
        except OSError:
            pass  # fail silently
        self.img_manager.set_prop(img_id, 'time_gen', data['time'])
        self.img_manager.set_prop(img_id, 'lat', data['latitude'])
        self.img_manager.set_prop(img_id, 'lng', data['longitude'])
        self.img_manager.set_prop(img_id, 'heading', data['heading'])
        self.img_manager.set_prop(img_id, 'altitude', data['altitude'])
        img = self.img_manager.get_img(img_id)
        self.img_manager.set_prop(img_id, 'width_px', img.shape[2])
        self.img_manager.set_prop(img_id, 'height_px', img.shape[1])
        self.task_queue.put(
            PriorityItem(2, {
                'type': 'auction',
                'event_name': 'yolo',
                'args': {
                    'img_id': img_id
                }
            }))

    def do_auction(self, *auctions):
        for auction in auctions:
            self.task_queue.put(
                PriorityItem(
                    2, {
                        'type': 'auction',
                        'event_name': auction['event_name'],
                        'args': auction['json']
                    }))


    def attach_listeners(self):
        self.socketio_server.on_event('connect',
                                      self.vision_socketio_server_connect)
        self.socketio_server.on_event('bid', self.receive_bid)
        self.socketio_server.on_event('process_image', self.process_image)
        self.socketio_server.on_event('download_complete', self.call_next)
        self.socketio_server.on_event('yolo_done', self.snip_img)
        self.socketio_server.on_event('snipped', self.download_snipped)
        self.socketio_server.on_event('classified', self.record_class)

        self.socketio_server.on_event('manual_request', self.manual_request)
        self.socketio_server.on_event('manual_request_done', 
                                      self.manual_request_done)
        self.socketio_server.on_event('get_all_images', self.return_all_images)
        self.socketio_server.on_event('calc_target_coords', 
                                      self.call_calc_target_coords)
        print('Listeners attached')

    
    # Functions to be attached to listeners

    def vision_socketio_server_connect(self):
        print("Someone connected to vision server")

    
    def receive_bid(self, json):
        self.s_worker.active_auctions[
            json['auction_id']]['bids'].append(json['bid'])


    # Step 1 - download the image
    def process_image(self, json, attempts=1):
        if self.verbose:
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
        img_inc_path = os.path.join(
            Config.DOCKER_DATA_DIR.value, img_info['id'])

        try:
            with open(img_inc_path + '.json', 'x') as f:
                json_module.dump(img_info, f)
        except FileExistsError:
            if manual_id:
                print('Manually Selected ID already exists: "{}"'.format(
                    img_info['id']))
                return
            if attempts <= 0:
                print('Process Image Error: Double image id collision for "{}" \
                    \n What are the odds? You should by a lottery ticket. \
                    '.format(img_info['id']))
                return
            # try again with a random image
            self.process_image(json, attempts - 1)
            return
        except OSError:
            # TODO handle OSError
            pass

        # TODO keep track of how many times rsync failed
        print("Telling rsync client to download image")
        # yapf: disable
        self.task_queue.put(PriorityItem(2, {
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
        self.task_queue.put(
            PriorityItem(
                1, {
                    'type':
                    'add_record',
                    'sql_statement':
                    "insert into Images values ('{}','{}')".format(
                        img_info['id'], 'raw')
                }))
        self.socketio_server.emit('new_raw', {'img_id': img_info['id']})
        self.img_count += 1


    # Intermediate step
    def call_next(self, json):
        self.funcs[json['next']['func']](*json['next']['args'])


    # Step 2 - find the targets in the image (yolo)

    # Step 3 - cut out those targets
    def snip_img(self, json):
        if self.verbose:
            print('Yolo finished; running snipper')
        self.task_queue.put(
            PriorityItem(
                2, {
                    'type': 'filter_and_snip',
                    'img_id': json['img_id'],
                    'yolo_results': json['results']
                }))


    # Step 4 - run shape classification on each target
    #      ... do letter classification and color recognition
    def download_snipped(self, json):
        img_id = json['img_id']
        download_dir = json['download_dir']
        print("Telling rsync client to download snipped image")
        # yapf: disable
        # WARNING: this auctions must occur as one event
        # to prevent duplicate "next" calls
        self.task_queue.put(PriorityItem(2, {
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
        self.task_queue.put(
            PriorityItem(
                1, {
                    'type':
                    'add_record',
                    'sql_statement':
                    "insert into Images values ('{}', 'localized')".format(img_id)
                }))
        self.socketio_server.emit('new_localized', json)


    def record_class(self, json):
        class_type = json['type']
        img_id = json['img_id']
        self.img_manager.set_prop(img_id, class_type, json[class_type])
        for check_class in ('shape', 'letter'):
            if self.img_manager.get_prop(img_id, check_class) is None:
                return
        # if it gets to this point without returning, then everything is done
        self.task_queue.put(
            PriorityItem(
                1, {
                    'type':
                    'add_record',
                    'sql_statement':
                    "update Images set Type = 'classified' where ImageID='{}'".
                    format(img_id)
                }))
        self.socketio_server.emit('new_classified', {'img_id': img_id})
        self.socketio_server.emit('image_processed', {'img_id': img_id})


    def manual_request(self, json):
        json['args']['manual'] = True
        self.task_queue.put(
            PriorityItem(
                0, {
                    'type': 'auction',
                    'event_name': json['event_name'],
                    'args': json['args']
                }))


    def manual_request_done(self, json):
        self.socketio_server.emit('manual_request_done', json)


    def return_all_images(self):
        self.task_queue.put(PriorityItem(0, {'type': 'retrieve_records'}))


    def call_calc_target_coords(self, json):
        lat, lng = coordinates.calculate_target_coordinates(
            target_pos_pixel=json['target_pixel_pos'],
            parent_img_real_coords=json['parent_img_real_coords'],
            parent_img_dimensions_pixel=json['parent_img_dimensions'],
            altitude=json['altitude'],
            heading=json['heading'])
        self.socketio_server.emit('found_target_coords', {
            'lat': lat,
            'lng': lng
        })
