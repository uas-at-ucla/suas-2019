from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import signal

app = Flask(__name__)
app.use_reloader = False

socketio = SocketIO(app)

def main():
    socketio.run(app, port = 8085, debug = False, log_output = False)

@socketio.on('connect')
def connect():
    print("Someone connected to drone_communications!")
    emit('connect_response')

@socketio.on('execute_commands')
def execute_commands(data):
    emit('execute_commands', data, broadcast=True, include_self=False)

@socketio.on('telemetry')
def broadcast_telemetry(data):
    emit('telemetry', data, broadcast=True, include_self=False)

@socketio.on('failsafe')
def failsafe():
    emit('failsafe', broadcast=True, include_self=False)

if __name__ == "__main__":
    main()