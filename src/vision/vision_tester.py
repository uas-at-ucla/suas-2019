import socketIO_client

if __name__ == '__main__':
    client = socketIO_client.SocketIO('0.0.0.0', 8099)
    print('connected')
    client.emit('process_image', {})
