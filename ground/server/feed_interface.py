import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

################################################################################
#                                                                              #
#           NOTE: ALL ERROR OUTPUT IS BEING PIPED TO /dev/null                 #
#                 IF THIS CODE HAS ERRORS, THEN THEY WILL NOT APPEAR!          #
#                                                                              #
################################################################################
import sys
sys.stderr = open('/dev/null', 'w')

USE_INTEROP = True

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
stopped = False
interop_client = None

if USE_INTEROP:
    interop_url='http://localhost:8000'
    interop_username='testuser'
    interop_password='testpass'
    interop_client = None
    for i in range(20):
        try:
            interop_client = interop.AsyncClient( \
                url=interop_url, \
                username=interop_username, \
                password=interop_password)
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
        try:
            missions = interop_client.get_missions().result()
            stationary_obstacles, moving_obstacles = \
                interop_client.get_obstacles().result()
            data = object_to_dict({ \
                'missions': missions, \
                'stationary_obstacles': stationary_obstacles, \
                'moving_obstacles': moving_obstacles, \
                'interop_connected': True})
            flask_socketio.emit('initial_data', data)
            return
        except:
            global interop_client
            interop_client = None
    flask_socketio.emit('initial_data', {'interop_connected': False})

@interface_socketio.on('connect_to_interop')
def connect_to_interop():
    if USE_INTEROP:
        try:
            global interop_client
            interop_client = interop.AsyncClient( \
                url=interop_url, \
                username=interop_username, \
                password=interop_password)
            flask_socketio.emit('interop_connected', True)
        except:
            flask_socketio.emit('interop_connected', False)      

@interface_socketio.on('moving_obstacles')
def broadcast_moving_obstacles(moving_obstacles):
    flask_socketio.emit('moving_obstacles', moving_obstacles, \
        broadcast=True, include_self=False)

@interface_socketio.on('execute_commands')
def send_commands(commands):
    communications.emit('execute_commands', commands)

@interface_socketio.on('interop_disconnected')
def interop_disconnected():
    flask_socketio.emit('interop_connected', False, \
        broadcast=True, include_self=False)

def refresh_moving_obstacles():
    # This is necessary to talk to the socket from a separate thread
    interface_client = socketIO_client.SocketIO('0.0.0.0', 8084)
    while True:
        time.sleep(1)
        if interop_client is not None:
            try:
                moving_obstacles = interop_client.get_obstacles().result()[1]
                interface_client.emit('moving_obstacles', \
                object_to_dict(moving_obstacles))
            except:
                global interop_client
                interop_client = None
                interface_client.emit('interop_disconnected')

def object_to_dict(my_object):
    return json.loads(json.dumps(my_object, default=lambda o: o.__dict__))

def on_telemetry(*args):
    telemetry = args[0]
    interface_socketio.emit("telemetry", telemetry)

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
                interface_socketio.emit('interop_connected', False)

def drone_connected():
    print "connected to drone"
    interface_socketio.emit('drone_connected')

def listen_for_communications():
    global communications
    communications = socketIO_client.SocketIO('0.0.0.0', 8085)
    communications.on('connect', drone_connected)
    communications.on('telemetry', on_telemetry)
    communications.wait()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    threading.Thread(target=listen_for_communications).start()
    if USE_INTEROP:
        threading.Thread(target=refresh_moving_obstacles).start()

    interface_socketio.run(app, '0.0.0.0', port = 8084)
