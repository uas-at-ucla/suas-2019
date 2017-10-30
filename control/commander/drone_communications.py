from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import signal

app = Flask(__name__)
app.use_reloader = False

socketio = SocketIO(app)

@socketio.on('connect')
def connect():
    print("Someone connected to drone_communications!")

@socketio.on('send_telemetry')
def broadcast_telemetry(data):
    emit('telemetry', data, broadcast=True, include_self=False)

def main():
    socketio.run(app, port = 8085)

if __name__ == "__main__":
    main()
