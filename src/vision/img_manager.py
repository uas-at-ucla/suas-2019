import os
import cv2
import sys
import uuid
import base64
import json
from pathlib import PurePosixPath, PureWindowsPath

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager


class ImgManager:
    def __init__(self, parent_dir, truth_src):
        '''
        parent_dir: str that represents the abs path to the working directory
        truth_src: {'user', 'addr', 'remote_dir', 'os_type'}
            user: username of the remote machine
            addr: ip address of the remote machine
            port: port for ssh
            remote_dir: the absolute path of the remote img directory
            os_type: type of os: 'windows' or 'posix'
        '''
        self.dir = parent_dir
        self.truth_src = truth_src
        self.procs = process_manager.ProcessManager()

        if truth_src['os_type'] == 'nt':
            self.remote_dir = PureWindowsPath(truth_src['remote_dir'])
        else:
            self.remote_dir = PurePosixPath(truth_src['remote_dir'])

    def stop(self):
        self.procs.killall()

    def gen_id():
        return base64.urlsafe_b64encode(
            uuid.uuid4().bytes).decode('utf-8')[:-2]

    def get_prop(self, img_id, prop, attempts=1):
        '''
        img_id: an image id defined above
        prop: the property to be accessed
        attempts: [OPTIONAL] used internally to keep track of how many attempts
                    to be made before giving up
        '''
        try:
            with open(os.path.join(self.dir, img_id + '.json'), 'r') as f:
                return json.load(f)[prop]
        except OSError as err:
            if attempts <= 0:
                raise err  # retrieval failed twice in a row!
            self._retrieve_img(img_id)
            return self.get_prop(img_id, prop, attempts - 1)  # try again
        except KeyError as err:
            return None

    def set_prop(self, img_id, prop, val, attempts=1):
        try:
            filename = os.path.join(self.dir, img_id + '.json')
            data = None
            with open(filename, 'r') as f:
                data = json.load(f)
            data[prop] = val
            with open(filename, 'w') as f:
                json.dump(data, f)
        except OSError as err:
            if attempts <= 0:
                raise err  # retrieval failed twice in a row!
            self._retrieve_img(img_id)
            return self.set_prop(img_id, prop, val, attempts - 1)

    def _retrieve_img(self, img_id):
        for ext in ('.jpg', '.json'):
            if 0 != processes.spawn_process_wait_for_code(
                    'rsync -vz --progress -e "ssh -p ' + str(
                        self.truth_src['port']) + '" "' +
                    self.truth_src['user'] + '@' + self.truth_src['addr'] +
                    ':' + str(self.remote_dir / (img_id + ext)) + '" ' +
                    os.path.join(self.dir, img_id + ext)):
                # TODO handle download failure
                pass

    def get_img(self, img_id, attempts=1):
        # check local dir first
        for f in os.listdir(self.dir):
            fsplit = f.split('.')
            if fsplit[0] == img_id and fsplit[1] == 'jpg':
                return cv2.imread(os.path.join(self.dir, f))

        # not in local dir, retrieve from original source of truth
        self._retrieve_img(img_id)
        return self.get_img(img_id, attempts - 1)

    def create_img(self, img, props):
        img_inc_path = os.path.join(self.dir, props['id'])

        # create the files
        try:
            with open(img_inc_path + '.json', 'x') as f:
                json.dump(props, f)
            cv2.imwrite(img_inc_path + '.jpg', img)
        except FileExistsError as err:
            if attempts <= 0:
                raise err  # what are the chances of two collisions in a row?
            # try again with a random image
            process_image(json, attempts - 1)
            return
        except OSError:
            # TODO handle OSError
            pass

    def create_new_img(self,
                       img,
                       img_id=None,
                       time_gen=None,
                       hash_type=None,
                       other={}):
        if img_id is None:
            img_id = self.gen_id()

        img_info = dict({
            'id': img_id,
            'time_gen': time.time(),
        }, **other)

        self.create_img(img, img_info)
