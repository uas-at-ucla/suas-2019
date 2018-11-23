import os
import socketIO_client
import uuid
import threading
import queue

from img_manager import ImgManager
from config import Config


class ClientWorker(threading.Thread):
    def __init__(self, in_q, socket_client, processes,
                 args, verbose=False):  # accept args anyway even if not used
        super(ClientWorker, self).__init__()
        self.in_q = in_q  # input queue (queue.Queue)
        self.stop_req = threading.Event()  # listen for a stop request
        self.vsn_user = args.vsn_user
        self.vsn_addr = args.vsn_addr
        self.vsn_port = args.vsn_port
        self.ssh_port = args.ssh_port
        self.data_dir = Config.DOCKER_DATA_DIR.value
        self.socket_client = socket_client
        self.processes = processes

        self.manager = ImgManager(
            Config.DOCKER_DATA_DIR.value, {
                'user': args.vsn_user,
                'addr': args.vsn_addr,
                'port': args.ssh_port,
                'remote_dir': args.remote_dir,
                'os_type': os.name
            })
        self.verbose = verbose

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


