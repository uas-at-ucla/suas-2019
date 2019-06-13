import zmq
import json

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.connect('tcp://localhost:5678')
sock.send_string(
    json.dumps({
        'image': '/path/to/image',
        'orientation': 'NW',
        'shape': 'Circle',
        'char': 'A',
        'shape_color': 'red',
        'char_color': 'blue',
        'lat': 12.3931,
        'lng': 45.1232
    }))
sock.recv_string()
