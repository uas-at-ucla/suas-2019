import os
import sys
import signal
import time
import json
import threading
from flask import Flask, render_template, request
import flask_socketio, socketIO_client
import base64

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

drone_sid = None
drone_ip = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'flappy'
ground_socketio_server = flask_socketio.SocketIO(app)
stopped = False
interop_client = None
missions = None
stationary_obstacles = None
moving_obstacles = None

commands = []

telemetry_num = 0;
drone_loop_num = 0;

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
@ground_socketio_server.on('join_room')
def connect(room):
    global drone_sid
    global drone_ip

    flask_socketio.join_room(room)
    print((room + " connected! (ip: " + request.remote_addr + ")"))

    if room == 'frontend':
        data = None
        if USE_INTEROP and interop_client is not None:
            data = object_to_dict({ \
                'missions': missions, \
                'stationary_obstacles': stationary_obstacles, \
                'moving_obstacles': moving_obstacles})
        else:
            data = {'interop_disconnected': True}

        if drone_sid:
            data['drone_connected'] = True

        flask_socketio.emit('initial_data', data)

    elif room == 'drone':
        drone_sid = request.sid;
        drone_ip = request.remote_addr;

        data = object_to_dict({ \
            'stationary_obstacles': stationary_obstacles, \
            'moving_obstacles': moving_obstacles})
        flask_socketio.emit('interop_data', \
            interop_data_to_obstacles_proto(data), room='drone')
        flask_socketio.emit('drone_connected', room='frontend')


@ground_socketio_server.on('disconnect')
def disconnect():
    global drone_sid
    if request.sid == drone_sid:
        flask_socketio.emit('drone_disconnected', room='frontend')
        print("drone disconnected!")
        drone_sid = None


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
def broadcast_interop_data(data):
    flask_socketio.emit('interop_data', data, \
        room='frontend')

    flask_socketio.emit('interop_data', \
        interop_data_to_obstacles_proto(data), room='drone')


@ground_socketio_server.on('drone_ping')
def send_ping_result(data):
    flask_socketio.emit('drone_ping', data, \
        room='frontend')


@ground_socketio_server.on('execute_commands')
def send_commands(data):
    flask_socketio.emit('drone_execute_commands', data, \
        room='drone')


@ground_socketio_server.on('set_state')
def set_state(data):
    flask_socketio.emit('drone_set_state', data, \
        room='drone')


@ground_socketio_server.on('request_commands')
def give_commands(options):
    flask_socketio.emit('commands_changed', {
        'commands': commands,
        'changedCommands': None,
        'restoreCommands': options['restoreCommands']
    })


@ground_socketio_server.on('commands_changed')
def commands_changed(data):
    global commands
    commands = data['commands']
    flask_socketio.emit('commands_changed', data, \
        room='frontend', include_self=False)


@ground_socketio_server.on('interop_disconnected')
def interop_disconnected():
    flask_socketio.emit('interop_disconnected', \
        room='frontend')


@ground_socketio_server.on('telemetry')
def telemetry(received_telemetry):
    global telemetry_num
    global drone_loop_num
    passed_messages = received_telemetry['message_index'] - telemetry_num;
    passed_loops = received_telemetry['loop_index'] - drone_loop_num;

    if passed_messages > 1:
        print("Lost " + str(passed_messages - 1) + " telemetry messages!")

    if passed_loops - passed_messages > 0:
        print("Drone skipped " + str(passed_loops - passed_messages) + " telemetry messages!")

    
    telemetry_num = received_telemetry['message_index']
    drone_loop_num = received_telemetry['loop_index']
    
    global interop_client

    sensors = received_telemetry['telemetry']['sensors']

    flask_socketio.emit('on_telemetry', received_telemetry, \
        room='frontend')

    if len(sensors) == 0:
        return

    if USE_INTEROP and interop_client is not None:
        lat = sensors['latitude']
        lng = sensors['longitude']
        alt = sensors['altitude']
        heading = sensors['heading']

        if all(val is not None for val in [lat, lng, alt, heading]):
            try:
                interop_telemetry = interop.Telemetry(lat, lng, alt, heading)
                interop_client.post_telemetry(interop_telemetry)
            except:
                interop_client = None
                flask_socketio.emit('interop_disconnected', room='frontend')


# Interop. #####################################################################
def object_to_dict(my_object):
    if my_object:
        return json.loads(json.dumps(my_object, default=lambda o: o.__dict__))
    return None


def get_interop_data():
    global stationary_obstacles
    global moving_obstacles
    global missions
    missions = interop_client.get_missions().result()
    stationary_obstacles, moving_obstacles = \
        interop_client.get_obstacles().result()


def auto_connect_to_interop():
    global interop_client
    # Default Interop
    interop_url = 'http://' + INTEROP_IP + ':8000'
    interop_username = 'testuser'
    interop_password = 'testpass'
    interop_client = None
    for i in range(20):
        try:
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

    while True:
        time.sleep(0.5)

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

    return base64.b64encode(obstacles.SerializeToString())


def ping_drone():
    # This is necessary to talk to the socket from a separate thread
    interface_client = socketIO_client.SocketIO('0.0.0.0', 8081)

    while True:
        time.sleep(1)
        if drone_ip:
            output = os.popen("ping -c 1 " + drone_ip).readlines()
            if len(output) >= 6:
                ms = output[5].split('= ')[1].split('/')[0]
                interface_client.emit('drone_ping', {'ms': ms});
            else:
                interface_client.emit('drone_ping', {'ms': None});


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    processes.spawn_process("npm start --silent --prefix ./interface/", None,
                            True, False)

    if AUTO_CONNECT_TO_INTEROP:
        t_connect = threading.Thread(target=auto_connect_to_interop)
        t_connect.daemon = True
        t_connect.start()

    if USE_INTEROP:
        if LOCAL_INTEROP:
            processes.spawn_process("python tools/run_interop.py", None, True,
                                    False)

        t = threading.Thread(target=refresh_interop_data)
        t.daemon = True
        t.start()


    t_ping = threading.Thread(target=ping_drone)
    t_ping.daemon = True
    t_ping.start()

    ground_socketio_server.run(app, '0.0.0.0', port=8081)
