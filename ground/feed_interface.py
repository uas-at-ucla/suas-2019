from flask import Flask, render_template
import flask_socketio, socketIO_client
import signal

app = Flask(__name__)
app.config['SECRET_KEY'] = 'flappy'
socketio = flask_socketio.SocketIO(app)

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

if __name__ == '__main__':
    communications = socketIO_client.SocketIO('0.0.0.0', 8085, \
            CommunicationsNamespace)

    signal.signal(signal.SIGINT, signal_received)

    socketio.run(app, port = 8084)
