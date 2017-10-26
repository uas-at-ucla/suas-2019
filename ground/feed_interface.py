from flask import Flask, render_template
from flask_socketio import SocketIO, emit

import json
import math

app = Flask(__name__)
app.config['SECRET_KEY'] = 'flappy'
socketio = SocketIO(app)

@socketio.on('connect')
def connect():
    print("Ground interface connected!")

if __name__ == '__main__':
    socketio.run(app)
