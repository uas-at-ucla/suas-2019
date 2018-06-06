import serial
import time
import threading
import signal
import sys
import os
import hashlib
import argparse
import zmq
os.chdir(os.path.dirname(os.path.realpath(__file__)))
sys.dont_write_bytecode = True

if os.uname()[4] != "armv7l":
    sys.path.insert(0, '../../lib')

import process_manager

processes = process_manager.ProcessManager()
processes.run_command("protoc -I. --proto_path=. " \
    + "--python_out=/tmp/ " \
    + "serial_comms_message.proto")
sys.path.insert(0, '/tmp')
import serial_comms_message_pb2 as serial_comms_message_proto

class GroundSerialComms:
    def __init__(self, device):
        self.ser = serial.Serial(
            port=device, \
            baudrate=57600, \
            parity=serial.PARITY_NONE, \
            stopbits=serial.STOPBITS_ONE, \
            bytesize=serial.EIGHTBITS, \
            timeout=0)

        self.stopped = False

    def run_reader(self, callback):
        self.read_callback = callback

        self.read_thread = \
                threading.Thread(target=self.read_data)
        self.read_thread.start()

    def read_data(self):
        time.sleep(0.001)
        while not self.stopped:
            buf = ""
            while True:
                try:
                    oneByte = self.ser.read(1)
                except:
                    break

                if oneByte == b"\r":
                    break
                else:
                    buf += oneByte

            self.parse_message(buf)

    def parse_message(self, raw_message):
        if len(raw_message) < 224 / 4:
            return

        checksum = raw_message[0:224 / 4]
        protobuf_encoded = raw_message[224 / 4 + 2:]

        calculated_checksum = hashlib.sha224(protobuf_encoded) \
                .hexdigest()

        if checksum != calculated_checksum:
            #print("Checksum mismatch")
            return

        proto_msg = serial_comms_message_proto.SerialCommsMessage()

        try:
            proto_msg.ParseFromString(protobuf_encoded)
        except:
            print("INVALID MESSAGE")
            return

        self.read_callback(proto_msg)

    def send_protobuf(self, proto_msg):
        proto_msg.unix_timestamp = time.time()

        protobuf_encoded = proto_msg.SerializeToString()
        checksum = hashlib.sha224(protobuf_encoded) \
                .hexdigest()

        message = checksum + "##" + protobuf_encoded + "\r"

        self.ser.write(message)

    def send_message(self, latitude, longitude, altitude, heading):
        proto_msg = serial_comms_message_proto.SerialCommsMessage()
        proto_msg.unix_timestamp = time.time()
        proto_msg.latitude = latitude
        proto_msg.longitude = longitude
        proto_msg.altitude = altitude
        proto_msg.heading = heading

        self.send_protobuf(proto_msg)

    def close(self):
        self.stopped = True
        self.ser.close()

ground_serial_comms = None

def signal_received(signal, frame):
    if ground_serial_comms != None:
        ground_serial_comms.close()

    sys.exit(0)

def run_sender(args):
    print("Running serial sender...")

    global ground_serial_comms
    ground_serial_comms = GroundSerialComms(args.device)

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, "")
    socket.bind("ipc:///tmp/serial_comms.ipc")
    print("CREATED ZMQ")

    time.sleep(1)

    while ground_serial_comms.ser.isOpen():
        time.sleep(0.2)
        serialized_protobuf = socket.recv()

        proto_msg = serial_comms_message_proto.SerialCommsMessage()

        try:
            proto_msg.ParseFromString(serialized_protobuf)
        except:
            print("INVALID MESSAGE")
            continue

        ground_serial_comms.send_protobuf(proto_msg)

    print("EXIT")

def print_received(proto_msg):
    print("Latitude: " + str(proto_msg.latitude) \
            + " Longitude: " + str(proto_msg.longitude) \
            + " Altitude: " + str(proto_msg.altitude) \
            + " Heading: " + str(proto_msg.heading))


def run_receiver(args):
    print("Running serial receiver...")

    global ground_serial_comms
    ground_serial_comms = GroundSerialComms(args.device)
    ground_serial_comms.run_reader(print_received)

    # Receive forever.
    while True:
        time.sleep(1)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_received)

    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(help='sub-command help')

    sender_parser = subparsers.add_parser('sender', \
            help='sender help')
    sender_parser.add_argument(
        '--device', action='store', help='device help', required=True)
    sender_parser.set_defaults(func=run_sender)

    receiver_parser = subparsers.add_parser('receiver', \
            help='receiver help')
    receiver_parser.add_argument(
        '--device', action='store', help='device help', required=True)
    receiver_parser.set_defaults(func=run_receiver)

    args = parser.parse_args()
    args.func(args)

