import threading
import signal
import time

try:
    from http.server import HTTPServer, SimpleHTTPRequestHandler # Python 3
except ImportError:
    from SimpleHTTPServer import BaseHTTPServer
    HTTPServer = BaseHTTPServer.HTTPServer
    from SimpleHTTPServer import SimpleHTTPRequestHandler # Python 2

server = HTTPServer(('0.0.0.0', 8080), SimpleHTTPRequestHandler)
thread = threading.Thread(target = server.serve_forever)
thread.daemon = True
thread.start()

def handle_signal(signum, frame):
    server.shutdown()

# Kill the server when this program is killed.
signal.signal(signal.SIGHUP, handle_signal)

while True:
    time.sleep(1)
