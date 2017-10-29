import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, './interop/client')

from flask import Flask, render_template
import flask_socketio, socketIO_client
import signal
import _thread
import interop

app = Flask(__name__)
app.config['SECRET_KEY'] = 'flappy'
socketio = flask_socketio.SocketIO(app)

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
    socketio.stop()

    sys.exit(0)

@socketio.on('connect')
def connect():
    print("Ground interface connected!")

class CommunicationsNamespace(socketIO_client.BaseNamespace):
    def on_connect():
        print('feed_interface connected to drone_communications!')

    def disconnect():
        print('Disconnected')

def on_telemetry(*args):
    print('Ground Station Received Telemetry!', args[0])
    #socketio.emit('telemetry', args[0])
    telemetry = interop.Telemetry(args[0]['lat'], args[0]['lng'], args[0]['alt'], args[0]['heading'])
    if interop_client:
        interop_client.post_telemetry(telemetry)

def listen_for_communications():
    communications = socketIO_client.SocketIO('0.0.0.0', 8085, \
            CommunicationsNamespace)
    communications.on('telemetry', on_telemetry)
    communications.wait()

if __name__ == '__main__':
    _thread.start_new_thread(listen_for_communications, ())

    signal.signal(signal.SIGINT, signal_received)

    socketio.run(app, port = 8084)