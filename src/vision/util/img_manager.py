import os
import cv2
import sys
import uuid
import base64
import json
import time
from pathlib import PurePosixPath, PureWindowsPath

sys.dont_write_bytecode = True
sys.path.insert(0, '../../lib')
import process_manager


class ImgManager:
    def __init__(self, parent_dir, truth_src=None, master=False):
        '''
        parent_dir: str that represents the abs path to the working directory
        truth_src: {'user', 'addr', 'remote_dir', 'os_type'}
            user: username of the remote machine
            addr: ip address of the remote machine
            port: port for ssh
            remote_dir: the absolute path of the remote img directory
            os_type: type of os: 'windows' or 'posix'
        '''
        self.master = master
        self.dir = os.path.realpath(parent_dir)
        self.truth_src = truth_src
        self.procs = process_manager.ProcessManager()

        if not master:
            if truth_src['os_type'] == 'nt':
                self.remote_dir = PureWindowsPath(truth_src['remote_dir'])
            else:
                self.remote_dir = PurePosixPath(truth_src['remote_dir'])

    def stop(self):
        self.procs.killall()

    def gen_id():
        nid = base64.urlsafe_b64encode(uuid.uuid4().bytes).decode('utf-8')[:-2]
        if nid[0] == '-':
            nid = '0' + nid[1:]
        return nid

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
            if attempts <= 0 or self.master:
                print('prop retrieval failed twice in a row!')
                return None
            self._update_props(img_id)
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
            return (prop, val)
        except OSError as err:
            if attempts <= 0 or self.master:
                print('prop retrieval failed twice in a row!')
                return None
            self._update_props(img_id)
            return self.set_prop(img_id, prop, val, attempts - 1)

    def _retrieve_img(self, img_id):
        if 0 != self.procs.spawn_process_wait_for_code(
                'rsync -vz --progress -e "ssh -p ' +
                str(self.truth_src['port']) + '" "' + self.truth_src['user'] +
                '@' + self.truth_src['addr'] + ':' + str(self.remote_dir /
                                                         (img_id + '.jpg')) +
                '" ' + os.path.join(self.dir, img_id + '.jpg')):
            pass # Do nothing on download failure

    def _update_props(self, img_id):
        if 0 != self.procs.spawn_process_wait_for_code(
                'rsync -vz --progress -e "ssh -p ' +
                str(self.truth_src['port']) + '" "' + self.truth_src['user'] +
                '@' + self.truth_src['addr'] + ':' + str(self.remote_dir /
                                                         (img_id + '.json')) +
                '" ' + os.path.join(self.dir, img_id + '.json.tmp')):
            pass # Do nothing on download failure
        else: # update image
            img_inc_path = os.path.join(self.dir, img_id)
            org_data = None
            data = None

            # Read the original data
            try:
                with open(img_inc_path + '.json', 'r') as f:
                    org_data = json.load(f)
            except OSError:
                org_data = {}

            # Read the new data
            try:
                with open(img_inc_path + '.json.tmp', 'r') as f:
                    data = json.load(f)
                os.remove(img_inc_path + '.json.tmp')
            except OSError:
                # This should never happen; download failure is handled earlier
                pass

            # Append/Overwrite the old data with the new data
            new_data = dict(org_data, **data)
            try:
                with open(img_inc_path + '.json', 'w') as f:
                    json.dump(new_data, f)
            except OSError:
                pass # Do nothing on OSError

    def get_img(self, img_id, attempts=1):
        # check local dir first
        for f in os.listdir(self.dir):
            fsplit = f.split('.')
            if fsplit[0] == img_id and fsplit[1] == 'jpg':
                return cv2.imread(os.path.join(self.dir, f))

        # not in local dir, retrieve from original source of truth
        if self.master:
            return None
        else:
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
                print('what are the chances of two collisions in a row?')
                return
            # try again with a random image
            process_image(json, attempts - 1)
            return
        except OSError:
            pass # Do nothing on OSError

    def create_new_img(self,
                       img,
                       img_id=None,
                       time_gen=None,
                       hash_type=None,
                       other={},
                       attempts=1):
        img_info = dict(
            {
                'id': img_id if img_id is not None else ImgManager.gen_id(),
                'time_gen': time_gen if time_gen is not None else time.time(),
            }, **other)

        img_inc_path = os.path.join(self.dir, img_info['id'])

        # create the files
        try:
            with open(img_inc_path + '.json', 'x') as f:
                json.dump(img_info, f)
            cv2.imwrite(img_inc_path + '.jpg', img)
            return img_info['id']
        except FileExistsError as err:
            if attempts <= 0:
                print('what are the chances of two collisions in a row?')
                return
            # try again with a random image
            return self.create_new_img(
                img,
                time_gen=img_info['time_gen'],
                hash_type=hash_type,
                other=other,
                attempts=attempts - 1)
        except OSError:
            pass # Do nothing on OS error
