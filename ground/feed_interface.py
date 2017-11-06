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
old_stdout, old_stderr = sys.stdout, sys.stderr
sys.stdout = open('/dev/null', 'w')
sys.stderr = open('/dev/null', 'w')

USE_INTEROP = True

sys.dont_write_bytecode = True

if USE_INTEROP:
    sys.path.insert(0, './interop/client')

from flask import Flask, render_template
import flask_socketio, socketIO_client
import signal
import time
import json
import _thread
if USE_INTEROP:
    import interop

app = Flask(__name__)
app.config['SECRET_KEY'] = 'flappy'
interface_socketio = flask_socketio.SocketIO(app, async_mode='threading')
stopped = False

if USE_INTEROP:
    interop_url='http://localhost:8000'
    interop_username='testuser'
    interop_password='testpass'
    try:
        interop_client = interop.AsyncClient( \
            url=interop_url, \
            username=interop_username, \
            password=interop_password)
    except Exception as e:
        print('Interop Server not running!')
        interop_client = None


def signal_received(signal, frame):
    global stopped
    stopped = True
    sys.exit(0)

@interface_socketio.on('connect')
def connect():
    print("Ground interface connected!")
    if USE_INTEROP and interop_client is not None:
        missions = interop_client.get_missions().result()
        stationary_obstacles, moving_obstacles = \
            interop_client.get_obstacles().result()
        data = object_to_dict({ \
            'missions': missions, \
            'stationary_obstacles': stationary_obstacles, \
            'moving_obstacles': moving_obstacles})
        flask_socketio.emit('missions_and_obstacles', data)

@interface_socketio.on('request_broadcast_moving_obstacles')
def broadcast_moving_obstacles(moving_obstacles):
    flask_socketio.emit('moving_obstacles', moving_obstacles, \
        broadcast=True, include_self=False)

def refresh_moving_obstacles():
    # This is necessary to talk to the socket from a separate thread
    interface_client = socketIO_client.SocketIO('0.0.0.0', 8084)
    while True:
        time.sleep(1)
        moving_obstacles = interop_client.get_obstacles().result()[1]
        interface_client.emit('request_broadcast_moving_obstacles', \
            object_to_dict(moving_obstacles))

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
            interop_telemetry = interop.Telemetry(lat, lng, alt, heading)
            interop_client.post_telemetry(interop_telemetry)

def listen_for_communications():
    communications = socketIO_client.SocketIO('0.0.0.0', 8085)
    communications.on('telemetry', on_telemetry)
    communications.wait()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    _thread.start_new_thread(listen_for_communications, ())
    if USE_INTEROP and interop_client is not None:
        _thread.start_new_thread(refresh_moving_obstacles, ())

    interface_socketio.run(app, '0.0.0.0', port = 8084)
