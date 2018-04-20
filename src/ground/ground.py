import os
import sys
import signal
import time
import json
import threading
from flask import Flask, render_template
import flask_socketio, socketIO_client

os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True
sys.path.insert(0, 'interop/client')
sys.path.insert(0, '../../lib')
import process_manager
import interop

sys.path.insert(0, './tools')

METERS_PER_FOOT = 0.3048

# Configuration options. #######################################################
USE_INTEROP = True
LOCAL_INTEROP = False
AUTO_CONNECT_TO_INTEROP = True
INTEROP_IP = '138.68.250.14'
if LOCAL_INTEROP:
    INTEROP_IP = '0.0.0.0'

app = Flask(__name__)
app.config['SECRET_KEY'] = 'flappy'
ground_socketio_server = flask_socketio.SocketIO(app)
stopped = False
interop_client = None
missions = None
stationary_obstacles = None
moving_obstacles = None

commands = []

processes = process_manager.ProcessManager()
processes.run_command("protoc -I. --proto_path=../../lib/mission_manager/ " \
    + "--python_out=/tmp/ " \
    + "../../lib/mission_manager/mission_commands.proto")
sys.path.insert(0, '/tmp')
import mission_commands_pb2 as proto


def signal_received(signal, frame):
    # Shutdown all the spawned processes and exit cleanly.
    processes.killall()
    stopped = True
    sys.exit(0)


# SocketIO handlers. ###########################################################
@ground_socketio_server.on('connect')
def connect():
    print("Someone connected!")
    flask_socketio.join_room('frontend')
    data = None
    if USE_INTEROP and interop_client is not None:
        data = object_to_dict({ \
            'missions': missions, \
            'stationary_obstacles': stationary_obstacles, \
            'moving_obstacles': moving_obstacles})
    else:
        data = {'interop_disconnected': True}
    flask_socketio.emit('initial_data', data)


@ground_socketio_server.on('connect_to_interop')
def connect_to_interop(data):
    global interop_client

    if USE_INTEROP:
        try:
            interop_username = 'testuser'
            interop_password = 'testpass'
            interop_url = 'http://' + str(data[0]) + '.' + str(data[1]) + \
            '.' + str(data[2]) + '.' + str(data[3]) + ':' + str(data[4])
            interop_client = interop.AsyncClient( \
                url=interop_url, \
                username=interop_username, \
                password=interop_password)
            get_interop_data()
            flask_socketio.emit('interop_data', object_to_dict({ \
                'missions': missions, \
                'stationary_obstacles': stationary_obstacles, \
                'moving_obstacles': moving_obstacles
            }), room='frontend')
        except:
            interop_client = None
            flask_socketio.emit('interop_disconnected', room='frontend')


@ground_socketio_server.on('interop_data')
def broadcast_obstacles(data):
    if 'frontend' in flask_socketio.rooms():
        flask_socketio.leave_room('frontend')

    flask_socketio.emit('interop_data', data, \
        room='frontend')

    flask_socketio.emit('interop_data', \
        interop_data_to_obstacles_proto(data), room='drone')


@ground_socketio_server.on('execute_commands')
def send_commands(data):
    flask_socketio.emit('drone_execute_commands', data, \
        room='drone')


@ground_socketio_server.on('set_state')
def send_commands(data):
    flask_socketio.emit('drone_set_state', data, \
        room='drone')


@ground_socketio_server.on('request_commands')
def give_commands():
    flask_socketio.emit('commands_changed', {
        'commands': commands,
        'changedCommands': None
    })


@ground_socketio_server.on('commands_changed')
def commands_changed(data):
    global commands
    commands = data['commands']
    flask_socketio.emit('commands_changed', data, \
        room='frontend', include_self=False)


@ground_socketio_server.on('interop_disconnected')
def interop_disconnected():
    if 'frontend' in flask_socketio.rooms():
        flask_socketio.leave_room('frontend')

    flask_socketio.emit('interop_disconnected', \
        room='frontend')


@ground_socketio_server.on('telemetry')
def telemetry(*args):
    if 'drone' not in flask_socketio.rooms():
        flask_socketio.leave_room('frontend')
        flask_socketio.join_room('drone')
        data = object_to_dict({ \
            'stationary_obstacles': stationary_obstacles, \
            'moving_obstacles': moving_obstacles})
        flask_socketio.emit('interop_data', \
            interop_data_to_obstacles_proto(data), room='drone')

    global interop_client

    received_telemetry = args[0]

    flask_socketio.emit('on_telemetry', received_telemetry, \
        room='frontend')

    if len(received_telemetry['sensors']) == 0:
        return

    if USE_INTEROP and interop_client is not None:
        lat = received_telemetry['sensors']['latitude']
        lng = received_telemetry['sensors']['longitude']
        alt = received_telemetry['sensors']['altitude']
        heading = received_telemetry['sensors']['heading']

        if all(val is not None for val in [lat, lng, alt, heading]):
            try:
                interop_telemetry = interop.Telemetry(lat, lng, alt, heading)
                interop_client.post_telemetry(interop_telemetry)
            except:
                interop_client = None
                flask_socketio.emit('interop_disconnected', room='frontend')


# Interop. #####################################################################
def object_to_dict(my_object):
    return json.loads(json.dumps(my_object, default=lambda o: o.__dict__))


def get_interop_data():
    global interop_client
    global stationary_obstacles
    global moving_obstacles
    global missions
    missions = interop_client.get_missions().result()
    stationary_obstacles, moving_obstacles = \
        interop_client.get_obstacles().result()


def auto_connect_to_interop():
    # Default Interop
    interop_url = 'http://' + INTEROP_IP + ':8000'
    interop_username = 'testuser'
    interop_password = 'testpass'
    interop_client = None
    for i in range(20):
        try:
            global stopped
            global interop_client

            if stopped:
                break
            interop_client = interop.AsyncClient( \
                url=interop_url, \
                username=interop_username, \
                password=interop_password)
            get_interop_data()
            print('Ground Station connected to Interop Server!')
            break
        except Exception as e:
            print(e)
            print('Waiting for Interop Server...')
            time.sleep(1)


def refresh_interop_data():
    # This is necessary to talk to the socket from a separate thread
    interface_client = socketIO_client.SocketIO('0.0.0.0', 8081)

    global interop_client
    global missions
    global stationary_obstacles
    global moving_obstacles

    while True:
        time.sleep(1)

        if interop_client is not None:
            try:
                old_stationary_obstacles = object_to_dict(stationary_obstacles)
                old_missions = object_to_dict(missions)

                get_interop_data()

                new_missions = object_to_dict(missions)
                send_missions = False
                if new_missions:
                    if len(new_missions) == len(old_missions):
                        for i in range(len(new_missions)):
                            if (json.dumps(new_missions[i]) != json.dumps(
                                    old_missions[i])):
                                send_missions = True
                                break
                    else:
                        send_missions = True

                new_stationary_obstacles = object_to_dict(stationary_obstacles)
                send_stationary_obstacles = False
                if new_stationary_obstacles:
                    if len(new_stationary_obstacles) == len(
                            old_stationary_obstacles):
                        for i in range(len(new_stationary_obstacles)):
                            if (json.dumps(new_stationary_obstacles[i]) !=
                                    json.dumps(old_stationary_obstacles[i])):
                                send_stationary_obstacles = True
                                break
                    else:
                        send_stationary_obstacles = True

                data = {}
                if moving_obstacles:
                    data['moving_obstacles'] = moving_obstacles
                if send_stationary_obstacles:
                    data['stationary_obstacles'] = stationary_obstacles
                if send_missions:
                    data['missions'] = missions

                interface_client.emit('interop_data', object_to_dict(data))
            except Exception as e:
                print(e)

                interop_client = None
                interface_client.emit('interop_disconnected')


def interop_data_to_obstacles_proto(data):
    obstacles = proto.Obstacles()
#   if 'moving_obstacles' in data:
#       for obstacle in data['moving_obstacles']:
#           proto_obstacle = obstacles.moving_obstacles.add()
#           proto_obstacle.sphere_radius = obstacle['sphere_radius'] \
#               * METERS_PER_FOOT
#           proto_obstacle.point.latitude = obstacle['latitude']
#           proto_obstacle.point.longitude = obstacle['longitude']
#           proto_obstacle.point.altitude = obstacle['altitude_msl']
    for obstacle in object_to_dict(stationary_obstacles):
        proto_obstacle = obstacles.static_obstacles.add()
        proto_obstacle.cylinder_radius = obstacle['cylinder_radius'] \
            * METERS_PER_FOOT
        proto_obstacle.location.latitude = obstacle['latitude']
        proto_obstacle.location.longitude = obstacle['longitude']

    return obstacles.SerializeToString().encode('base64')


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    processes.spawn_process("npm start --silent --prefix ./interface/", None,
                            True, False)

    if AUTO_CONNECT_TO_INTEROP:
        t = threading.Thread(target=auto_connect_to_interop)
        t.daemon = True
        t.start()

    if USE_INTEROP:
        if LOCAL_INTEROP:
            processes.spawn_process("python tools/run_interop.py", None, True,
                                    False)

        t = threading.Thread(target=refresh_interop_data)
        t.daemon = True
        t.start()

    ground_socketio_server.run(app, '0.0.0.0', port=8081)
