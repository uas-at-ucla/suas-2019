from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import signal

app = Flask(__name__)
app.use_reloader = False

socketio = SocketIO(app)

def signal_received(signal, frame):
    socketio.stop()

    sys.exit(0)

@socketio.on('connect')
def connect():
    print("Someone connected to drone_communications!")

def main():
    signal.signal(signal.SIGINT, signal_received)

    socketio.run(app, port = 8085)

if __name__ == "__main__":
    main()
