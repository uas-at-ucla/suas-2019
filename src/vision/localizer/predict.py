from darkflow.net.build import TFNet
import cv2
import random
import signal
import sys
import time

def randomColor():
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

def signal_received(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_received)

options = {
    "pbLoad": "darkflow/built_graph/yolo-auvsi.pb",
    "metaLoad": "darkflow/built_graph/yolo-auvsi.meta",
    "threshold": 0.001
}

tfnet = TFNet(options)

img = cv2.imread(sys.argv[1])

results = tfnet.return_predict(img)

for result in results:
    color = randomColor()
    topLeft = result['topleft']
    botRight = result['bottomright']
    coords = {
        'topLeft': (topLeft['x'], topLeft['y']),
        'botRight': (botRight['x'], botRight['y'])
    }
    cv2.putText(img, result['label'], coords['topLeft'], cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
    cv2.rectangle(img, coords['topLeft'], coords['botRight'], color, 1)

print("Results:")
print(results)
cv2.imshow("img", img)
cv2.waitKey(1)
while (True):
    time.sleep(0.5)
cv2.destroyAllWindows()
