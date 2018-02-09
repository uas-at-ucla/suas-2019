import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

import sys

USE_INTEROP = True
LOCAL_INTEROP = False

interop_ip = '138.68.250.14'
if LOCAL_INTEROP:
    interop_ip = 'localhost'

# Enable True for localhost; False to use another ip address
AUTO_CONNECT_TO_INTEROP = True

sys.dont_write_bytecode = True

if USE_INTEROP:
    sys.path.insert(0, '../interop/client')

from flask import Flask, render_template
import flask_socketio, socketIO_client
import signal
import time
import json
import threading
if USE_INTEROP:
    import interop

app = Flask(__name__)
app.config['SECRET_KEY'] = 'flappy'
interface_socketio = flask_socketio.SocketIO(app, async_mode='threading')
communications = None
stopped = False
interop_client = None
missions = None
stationary_obstacles = None
moving_obstacles = None

def get_interop_data():
    global missions
    global stationary_obstacles
    global moving_obstacles
    missions = interop_client.get_missions().result()
    stationary_obstacles, moving_obstacles = \
        interop_client.get_obstacles().result()

if AUTO_CONNECT_TO_INTEROP:
    # Default Interop
    interop_url = 'http://' + interop_ip + ':8000'
    interop_username = 'testuser'
    interop_password = 'testpass'
    interop_client = None
    for i in range(20):
        try:
            interop_client = interop.AsyncClient( \
                url=interop_url, \
                username=interop_username, \
                password=interop_password)
            get_interop_data()
            print('Ground Station connected to Interop Server!')
            break
        except Exception as e:
            print('Waiting for Interop Server...')
            time.sleep(1)


def signal_received(signal, frame):
    global stopped
    stopped = True
    sys.exit(0)


@interface_socketio.on('connect')
def connect():
    print("Ground interface connected!")
    if USE_INTEROP and interop_client is not None:
        data = object_to_dict({ \
            'missions': missions, \
            'stationary_obstacles': stationary_obstacles, \
            'moving_obstacles': moving_obstacles})
        flask_socketio.emit('initial_data', data)
    else:
        flask_socketio.emit('initial_data', {'interop_disconnected': True})


@interface_socketio.on('connect_to_interop')
def connect_to_interop(data):
    if USE_INTEROP:
        try:
            interop_username = 'testuser'
            interop_password = 'testpass'
            interop_url = 'http://' + str(data[0]) + '.' + str(data[1]) + \
            '.' + str(data[2]) + '.' + str(data[3]) + ':' + str(data[4])
            global interop_client
            interop_client = interop.AsyncClient( \
                url=interop_url, \
                username=interop_username, \
                password=interop_password)
            get_interop_data()
            flask_socketio.emit('interop_data', object_to_dict({ \
                'missions': missions, \
                'stationary_obstacles': stationary_obstacles, \
                'moving_obstacles': moving_obstacles
            }))
        except:
            global interop_client
            interop_client = None
            flask_socketio.emit('interop_disconnected')

@interface_socketio.on('interop_data')
def broadcast_obstacles(data):
    flask_socketio.emit('interop_data', data, \
        broadcast=True, include_self=False)
    if communications:
        communications.emit('interop_data', data)


@interface_socketio.on('execute_commands')
def send_commands(commands):
    if communications:
        communications.emit('execute_commands', commands)


@interface_socketio.on('set_state')
def send_commands(command):
    if communications:
        communications.emit('set_state', command)


@interface_socketio.on('test_image')
def request_test_image(data):
    if communications:
        communications.emit('test_image', data)


@interface_socketio.on('interop_disconnected')
def interop_disconnected():
    flask_socketio.emit('interop_disconnected', \
        broadcast=True, include_self=False)


def refresh_interop_data():
    # This is necessary to talk to the socket from a separate thread
    interface_client = socketIO_client.SocketIO('0.0.0.0', 8084)
    while True:
        time.sleep(1)
        if interop_client is not None:
            try:
                old_stationary_obstacles = object_to_dict(stationary_obstacles)
                old_missions = object_to_dict(missions)

                get_interop_data()

                new_missions = object_to_dict(missions)
                send_missions = False
                if len(new_missions) == len(old_missions):
                    for i in range(len(new_missions)):
                        if (json.dumps(new_missions[i]) != json.dumps(old_missions[i])):
                            send_missions = True
                            break
                else:
                    send_missions = True

                new_stationary_obstacles = object_to_dict(stationary_obstacles)
                send_stationary_obstacles = False
                if len(new_stationary_obstacles) == len(old_stationary_obstacles):
                    for i in range(len(new_stationary_obstacles)):
                        if (json.dumps(new_stationary_obstacles[i]) != json.dumps(old_stationary_obstacles[i])):
                            send_stationary_obstacles = True
                            break
                else:
                    send_stationary_obstacles = True

                data = {'moving_obstacles': moving_obstacles}
                if send_stationary_obstacles:
                    data['stationary_obstacles'] = stationary_obstacles
                if send_missions:
                    data['missions'] = missions

                interface_client.emit('interop_data', object_to_dict(data))
            except:
                global interop_client
                interop_client = None
                interface_client.emit('interop_disconnected')


def object_to_dict(my_object):
    return json.loads(json.dumps(my_object, default=lambda o: o.__dict__))


def on_telemetry(*args):
    telemetry = args[0]
    interface_socketio.emit('telemetry', telemetry)

    if USE_INTEROP and interop_client is not None:
        lat = telemetry['gps_lat']
        lng = telemetry['gps_lng']
        alt = telemetry['gps_alt']
        heading = telemetry['heading']
        if all(val is not None for val in [lat, lng, alt, heading]):
            try:
                interop_telemetry = interop.Telemetry(lat, lng, alt, heading)
                interop_client.post_telemetry(interop_telemetry)
            except:
                global interop_client
                interop_client = None
                interface_socketio.emit('interop_disconnected')


def on_image(*args):
    interface_socketio.emit('image', args[0])


def drone_connected():
    print "connected to drone"
    interface_socketio.emit('drone_connected')
    communications.emit('interop_data', object_to_dict({ \
        'missions': missions, \
        'stationary_obstacles': stationary_obstacles, \
        'moving_obstacles': moving_obstacles \
    }))


def drone_disconnected():
    print "disconnected from drone!"
    communications.disconnect()
    interface_socketio.emit('drone_disconnected')
    listen_for_communications()


def listen_for_communications():
    global communications
    communications = socketIO_client.SocketIO('0.0.0.0', 8085)
    communications.on('connect', drone_connected)
    communications.once('disconnect', drone_disconnected)
    communications.on('telemetry', on_telemetry)
    communications.on('image', on_image)
    communications.wait()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    t = threading.Thread(target=listen_for_communications)
    t.daemon = True
    t.start()
    if USE_INTEROP:
        t = threading.Thread(target=refresh_interop_data)
        t.daemon = True
        t.start()

    interface_socketio.run(app, '0.0.0.0', port=8084)
