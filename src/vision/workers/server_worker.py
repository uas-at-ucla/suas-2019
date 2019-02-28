import time
import uuid
import sqlite3
import threading
import queue

# from dataclasses import dataclass, field
# from types import Any

from config import Config
import coordinates

# use this once we have python 3.7
# @dataclass(order=True)
# class PriorityItem:
#     key: int
#     item: Any = field(compare=False)
class PriorityItem:
    def __init__(self, key, item):
        self.key = key
        self.item = item

    def __lt__(self, other):
        return self.key < other.key

    def __eq__(self, other):
        return self.key == other.key


class ServerWorker(threading.Thread):
    # accept args anyway even if not used
    def __init__(self, in_q, sio_srv, img_manager,
                 active_auctions=None, taken_auctions=None):
        super(ServerWorker, self).__init__()
        self.in_q = in_q  # input queue (queue.Queue)
        self.stop_req = threading.Event()  # listen for a stop request
        self.sio_srv = sio_srv
        self.img_manager = img_manager
        if active_auctions is None:
            self.active_auctions = {}
        else:
            self.active_auctions = active_auctions
        if taken_auctions is None:
            self.taken_auctions = {}
        else:
            self.taken_auctions = taken_auctions

        # Initialize SQL connection
        sql_connection = sqlite3.connect(Config.DOCKER_DATA_DIR.value +
                                         '/image_info.db')
        sql_cursor = sql_connection.cursor()
        with sql_connection:
            sql_cursor.execute(
                'create table if not exists Images (ImageID text, Type text)')
            sql_cursor.execute('create table if not exists Locations \
                (ImageID text, Lat real, Lng real)')
        sql_connection.close()


    def run(self):
        while not self.stop_req.isSet():  # Exit run if stop was requested
            try:
                # Try to get an item from the queue
                # blocking: true; timeout: 0.05
                task = self.in_q.get(True, 0.05).item

                ############ Task Format ############# # noqa: E266
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
                # task = {
                #     'type': 'add_record',
                #     'sql_statement': str
                # }
                # task = {
                #     'type': 'retrieve_records'
                # }

                task_type = task['type']
                if task_type == 'add_record':
                    self._add_record(task)
                elif task_type == 'retrieve_records':
                    self._retrieve_records(task)
                # check on the progress of an auction
                elif task_type == 'timeout':
                    self._timeout(task)
                # create a new auction
                elif task_type == 'auction':
                    self._auction(task)
                elif task_type == 'filter_and_snip':
                    self._filter_and_snip(task)

            except queue.Empty:
                continue


    def _add_record(self, task):
        sql_connection = sqlite3.connect(Config.DOCKER_DATA_DIR.value +
                                         '/image_info.db')
        sql_cursor = sql_connection.cursor()
        with sql_connection:
            sql_cursor.execute(task['sql_statement'])
        sql_connection.close()


    def _retrieve_records(self, task):
        sql_connection = sqlite3.connect(Config.DOCKER_DATA_DIR.value +
                                         '/image_info.db')
        sql_cursor = sql_connection.cursor()
        all_images = {'raw': None, 'localized': None, 'classified': None}
        with sql_connection:
            for img_type in ('raw', 'localized', 'classified'):
                sql_cursor.execute(
                    "select ImageID from Images where Type=?",
                    (img_type, ))
                all_images[img_type] = [
                    row[0] for row in sql_cursor.fetchall()
                ]
        self.sio_srv.emit('all_images', all_images)
        sql_connection.close()


    def _timeout(self, task):
        if (time.time() - task['time_began']
            ) >= Config.DEFAULT_AUCTION_TIMEOUT.value:
            auction = self.active_auctions[task['auction_id']]

            # reset the timer if no bids have been made
            if len(auction['bids']) == 0:
                task['time_began'] = time.time()
                self.in_q.put(PriorityItem(2, task))
            else:
                # choose the lowest bidder
                lowest_bid = auction['bids'][0]
                for bid in auction['bids']:
                    if bid['workload'] < lowest_bid['workload']:
                        lowest_bid = bid

                # announce the winner of the auction
                self.sio_srv.emit(
                    auction['event_name'] + '_' +
                    lowest_bid['client_id'], auction['args'])

                # remove the auction entry
                self.taken_auctions[
                    task['auction_id']] = self.active_auctions.pop(
                        task['auction_id'])
                # check on the progress of the task TODO
        else:
            # check again later since not enough time passed
            self.in_q.put(PriorityItem(2, task))


    def _auction(self, task):
        # generate a random auction id
        auction_id = str(uuid.uuid4())
        # announce the auction to all clients
        self.sio_srv.emit(task['event_name'], {'auction_id': auction_id})
        # add a new auction listing
        self.active_auctions[auction_id] = {
            'bids': [],
            'event_name': task['event_name'],
            'args': task['args']
        }
        # check in on this auction at a later time
        self.in_q.put(
            PriorityItem(
                2, {
                    'type': 'timeout',
                    'time_began': time.time(),
                    'auction_id': auction_id
                }))


    def _filter_and_snip(self, task):
        sql_connection = sqlite3.connect(Config.DOCKER_DATA_DIR.value +
                                         '/image_info.db')
        sql_cursor = sql_connection.cursor()
        results = task['yolo_results']
        img_id = task['img_id']
        filtered_results = []
        with sql_connection:
            # yapf: disable
            real_coords = (
                self.img_manager.get_prop(img_id, 'lat'),
                self.img_manager.get_prop(img_id, 'lat'))
            img_dimensions = (
                self.img_manager.get_prop(img_id, 'width_px'),
                self.img_manager.get_prop(img_id, 'height_px'))
            altitude = self.img_manager.get_prop(img_id, 'altitude')
            heading = self.img_manager.get_prop(img_id, 'heading')

            for result in results:
                target_pos = (
                    result['bottomright']['x'] - result['topleft']['x'],
                    result['bottomright']['y'] - result['topleft']['y'])
                lat, lng = coordinates.calculate_target_coordinates(
                    target_pos_pixel=target_pos,
                    parent_img_real_coords=real_coords,
                    parent_img_dimensions_pixel=img_dimensions,
                    altitude=altitude,
                    heading=heading)
                # yapf: enable

                # This will select any target within ~15m square of
                # (lat, lng) at the latitude of the Andrews Base
                sql_cursor.execute(
                    "select ImageID from Locations where (Lat \
                    between ? and ?) and (Lng between ? and ?)",
                    (lat - 0.01, lat + 0.01, lng - 0.01, lng + 0.01))
                if sql_cursor.fetchone() is None:
                    sql_cursor.execute(
                        "insert into Locations values (?, ?, ?)",
                        (img_id, lat, lng))
                    result['coords'] = {'lat': lat, 'lng': lng}
                    filtered_results.append(result)

        if len(filtered_results) > 0:
            self.in_q.put(
                PriorityItem(
                    2, {
                        'type': 'auction',
                        'event_name': 'snip',
                        'args': {
                            'img_id': task['img_id'],
                            'yolo_results': filtered_results
                        }
                    }))
        sql_connection.close()


    def join(self, timeout=None):
        self.stop_req.set()
        super().join(timeout)
