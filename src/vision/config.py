import os
from enum import Enum


class CONFIG(Enum):
    # General Defaults
    DOCKER_DATA_DIR = os.path.abspath('data_local')

    # Camera Defaults
    CAMERA_SENSOR_DIMENSIONS = (22.3, 14.9)  # mm
    LENS_FOCAL_LENGTH = 18  # mm

    # Server defaults
    DRONE_HOST_KEY = '/suas/src/vision/known_hosts'
    DRONE_SSH_ID = '/suas/src/vision/id_rsa_local'
    DRONE_USER = 'benlimpa'
    SECRET_KEY = 'flappy'
    DEFAULT_SRV_IP = '0.0.0.0'
    DEFAULT_SRV_PORT = 8099
    DRONE_IP = '0.0.0.0'
    YOLO_IP = '0.0.0.0'
    RSYNC_IP = '0.0.0.0'
    SNIPPER_IP = '0.0.0.0'
    CLASSIFIER_IP = '0.0.0.0'
    SNIPPER_HOST_KEY = '/suas/src/vision/known_hosts'
    SNIPPER_SSH_ID = '/suas/src/vision/id_rsa_local'
    SNIPPER_USER = 'benlimpa'
    DRONE_IMG_FOLDER = '/path/to/images'

    # Client Defaults
    DEFAULT_VSN_USER = 'benlimpa'
    DEFAULT_VSN_PORT = DEFAULT_SRV_PORT
    DEFAULT_SSH_PORT = 22
    DEFAULT_REMOTE_DIR = DOCKER_DATA_DIR
    DEFAULT_VSN_IP = DEFAULT_SRV_IP
    DEFAULT_VSN_SRV = DEFAULT_SRV_IP + ':' + str(DEFAULT_SRV_PORT)
    DEFAULT_THREADS = 1
    DEFAULT_AUCTION_TIMEOUT = 0.3
    MAX_THREADS = 20

    # Yolo Defaults
    DEFAULT_YOLO_PB = 'localizer/built_graph/yolo-auvsi.pb'
    DEFAULT_YOLO_META = 'localizer/built_graph/yolo-auvsi.meta'
    DEFAULT_YOLO_THRESH = 0.0012
