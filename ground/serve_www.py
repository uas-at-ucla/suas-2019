import threading
import signal
import time
import os
try:
    # Python 3
    from http.server import HTTPServer, SimpleHTTPRequestHandler
except ImportError:
    # Python 2
    from SimpleHTTPServer import BaseHTTPServer
    HTTPServer = BaseHTTPServer.HTTPServer
    from SimpleHTTPServer import SimpleHTTPRequestHandler

# CONSTANTS ####################################################################
SERVER_DOMAIN = "0.0.0.0"
SERVER_PORT = 8080
WWW_DIRECTORY_RELATIVE_LOCATION = "www"
################################################################################

# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

# Go into the www folder so that we are serving those files.
os.chdir(WWW_DIRECTORY_RELATIVE_LOCATION)

server = HTTPServer((SERVER_DOMAIN, SERVER_PORT), SimpleHTTPRequestHandler)
thread = threading.Thread(target = server.serve_forever)
thread.daemon = True
thread.start()

def handle_signal(signum, frame):
    server.shutdown()

# Kill the server when this program is killed.
signal.signal(signal.SIGHUP, handle_signal)

print("UAS at UCLA ground interface successfully started!")
print("Access the interface at http://" + SERVER_DOMAIN + ":" + \
        str(SERVER_PORT) + "/")

while True:
    time.sleep(1)
