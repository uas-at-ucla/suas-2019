from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import signal

import os
dname = os.path.dirname(os.path.realpath(__file__))

app = Flask(__name__)
app.use_reloader = False

socketio = SocketIO(app)


def main():
    print("Starting drone_communications")
    socketio.run(app, '0.0.0.0', port=8085, debug=False, log_output=False)


@socketio.on('connect')
def connect():
    print("Someone connected to drone_communications!")
    emit('connect_response')


@socketio.on('execute_commands')
def execute_commands(data):
    emit('execute_commands', data, broadcast=True, include_self=False)


@socketio.on('set_state')
def set_state(data):
    emit('set_state', data, broadcast=True, include_self=False)


@socketio.on('telemetry')
def broadcast_telemetry(data):
    # print "TELEMETRY"
    emit('telemetry', data, broadcast=True, include_self=False)


IMAGE_FOLDER = '../../vision/photos/'


@socketio.on('image')
def broadcast_image(data):
    with open(
            os.path.join(dname, IMAGE_FOLDER + data['fileName']),
            mode='rb') as file:
        image = file.read()
    data['base64'] = image.encode("base64")
    emit('image', data, broadcast=True, include_self=False)


@socketio.on('test_image')
def broadcast_image(data):
    with open(
            os.path.join(dname, IMAGE_FOLDER + data['fileName']),
            mode='rb') as file:
        image = file.read()
    data['base64'] = image.encode("base64")
    emit('image', data)


if __name__ == "__main__":
    main()
