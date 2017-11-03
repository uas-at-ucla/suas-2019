import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

import sys

USE_INTEROP = False

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
        missions_json = json.dumps(missions, default=lambda o: o.__dict__)
        print 'Missions: ' + str(missions_json)
        flask_socketio.emit('missions', missions_json)
        # stationary_obstacles, moving_obstacles = interop_client.get_obstacles()

def on_telemetry(*args):
    telemetry = args[0]
    print('Ground Station Received Telemetry: ' + str(telemetry))
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

    interface_socketio.run(app, '0.0.0.0', port = 8084)
