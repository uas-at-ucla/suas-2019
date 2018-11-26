import socketIO_client
import uuid
import queue


class VisionClient:
    def __init__(self, args, worker_class, processes, verbose):
        self.id = str(uuid.uuid4())
        self.work_queue = queue.Queue()
        self.client_workers = []
        self.processes = processes
        self.verbose = verbose

        # Connect to vision server
        print('Attempting to connect to server @ ' + args.vsn_addr + ':' +
              str(args.vsn_port))
        self.socket_client = socketIO_client.SocketIO(args.vsn_addr, 
                                                      port=args.vsn_port)
        print('Connected to server!')

        # initialize worker and listen for tasks
        for i in range(0, args.threads):
            c_worker = worker_class(in_q=self.work_queue, 
                                    socket_client=self.socket_client, 
                                    processes=processes, args=args, 
                                    verbose=verbose)
            c_worker.start()
            self.client_workers.append(c_worker)
        self.socket_client.on(self.client_workers[0].get_event_name(), 
                              self._bid_for_task)
        self.socket_client.on(self.client_workers[0].get_event_name() 
                              + '_' + self.id, self._add_task)
        self.socket_client.wait()


    def kill_workers(self):
        for worker in self.client_workers:
            worker.join()


    def _add_task(self, *args):
        self.work_queue.put(args)


    def _bid_for_task(self, *args):
        auction = args[0]
        self.socket_client.emit(
            'bid', {
                'auction_id': auction['auction_id'],
                'bid': {
                    'client_id': self.id,
                    'workload': self.work_queue.qsize()
                }
            })
