import time
import uuid
import sqlite3
import numpy as np
import math
import threading
import queue

from config import CONFIG


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
    def __init__(self,
                 in_q,
                 sio_srv,
                 active_auctions=None,
                 taken_auctions=None):
        super(ServerWorker, self).__init__()
        self.in_q = in_q  # input queue (queue.Queue)
        self.stop_req = threading.Event()  # listen for a stop request
        self.sio_srv = sio_srv
        if active_auctions is None:
            self.active_auctions = []
        else:
            self.active_auctions = active_auctions
        if taken_auctions is None:
            self.take_auctions = []
        else:
            self.taken_auctions = taken_auctions

    def run(self):
        sql_connection = sqlite3.connect(CONFIG.DOCKER_DATA_DIR.value +
                                         '/image_info.db')
        sql_cursor = sql_connection.cursor()
        with sql_connection:
            sql_cursor.execute(
                'create table if not exists Images (ImageID text, Type text)')
            sql_cursor.execute('create table if not exists Locations \
                (ImageID text, Lat real, Lng real)')

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
                    with sql_connection:
                        sql_cursor.execute(task['sql_statement'])
                elif task_type == 'retrieve_records':
                    all_images = {
                        'raw': None,
                        'localized': None,
                        'classified': None
                    }
                    with sql_connection:
                        for img_type in ('raw', 'localized', 'classified'):
                            sql_cursor.execute(
                                "select ImageID from Images where Type=?",
                                (img_type, ))
                            all_images[img_type] = [
                                row[0] for row in sql_cursor.fetchall()
                            ]
                    self.sio_srv.emit('all_images', all_images)

                # check on the progress of an auction
                elif task_type == 'timeout':
                    if (time.time() - task['time_began']
                        ) >= CONFIG.DEFAULT_AUCTION_TIMEOUT.value:
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

                # create a new auction
                elif task_type == 'auction':
                    # generate a random auction id
                    auction_id = str(uuid.uuid4())

                    # announce the auction to all clients
                    self.sio_srv.emit(task['event_name'],
                                      {'auction_id': auction_id})

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
                elif task_type == 'filter_and_snip':
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
                        altitude = self.img_manager.get_prop(
                            img_id, 'altitude')
                        heading = self.img_manager.get_prop(
                            img_id, 'heading')
                        for result in results:
                            target_pos = (
                                result['bottomright']['x']
                                - result['topleft']['x'],
                                result['bottomright']['y']
                                - result['topleft']['y'])

                            lat, lng = calculate_target_coordinates(
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
                                (lat - 0.01, lat + 0.01, lng - 0.01,
                                 lng + 0.01))
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

            except queue.Empty:
                continue
        else:
            sql_connection.close()

    def join(self, timeout=None):
        self.stop_req.set()
        super().join(timeout)


def calculate_target_coordinates(
        target_pos_pixel,
        parent_img_real_coords,
        parent_img_dimensions_pixel,
        altitude,
        heading,
        focal_length=CONFIG.LENS_FOCAL_LENGTH.value,
        sensor_dimensions=CONFIG.CAMERA_SENSOR_DIMENSIONS.value):
    """ Calculate the coordinates of a target in an image.

    Arguments:
    target_pos_pixel -- (pixels) the tuple(x, y) position of the target (top-left origin)
    parent_img_real_coords -- (ISO 6709, degrees) the (lat, lng) position of the center of the image
    parent_img_dimensions_pixel -- (pixels) the dimensions of the image
    altitude -- (m) the altitude when the image was taken
    heading -- (+CW degrees) the direction of the top of the picture
    sensor_dimensions -- (mm) the dimensions of the sensor !!! ratio must match image !!!
    focal_length -- (mm) the focal_length of the lens
    """  # noqa
    # yapf: disable

    # the algorithm assumes positive degrees are CCW
    # but the heading is given with +CW
    heading = -heading

    # get the average of ratio from the two dimensions
    pixel_to_sensor_ratio = 0
    for pixel_dim in parent_img_dimensions_pixel:
        for sensor_dim in sensor_dimensions:
            pixel_to_sensor_ratio += sensor_dim / pixel_dim
    pixel_to_sensor_ratio /= 2

    scaling_ratio = altitude / focal_length * pixel_to_sensor_ratio
    parent_img_center = tuple(int(component / 2)
                              for component in parent_img_dimensions_pixel)
    target_vec = np.array([
        (scaling_ratio * (parent_img_center[0] - target_pos_pixel[0])),
        (scaling_ratio * (parent_img_center[1] - target_pos_pixel[1]))
        ],
        dtype=np.float64)
    rotation_matrix = np.array([[math.cos(heading), -math.sin(heading)],
                                [math.sin(heading),  math.cos(heading)]])
    # yapf: enable
    target_vec = rotation_matrix @ target_vec
    new_lat = parent_img_real_coords[0] + (
        -target_vec[1] / CONFIG.R_EARTH.value) * 180 / math.pi
    # this is an approximation assuming the latitude remains constant
    # or is very small compared to the radius of the earth
    new_lng = parent_img_real_coords[1] + (target_vec[0] / (
        CONFIG.R_EARTH.value / math.cos(new_lat))) * 180 / math.pi
    return (new_lat, new_lng)
