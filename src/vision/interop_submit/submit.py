import zmq
import json


def submit_image(img_data):
    print(img_data)
    # TODO: For andrew: communicate with the interop server to submit images


if __name__ == '__main__':
    context = zmq.Context()
    cmd_listener = context.socket(zmq.REP)
    cmd_listener.bind(f'tcp://*:5678')

    while True:
        message = cmd_listener.recv_string()
        img_data = json.loads(message)
        submit_image(img_data)
        cmd_listener.send_string(json.dumps({'result': 'submitted'}))
