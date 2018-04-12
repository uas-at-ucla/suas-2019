from darkflow.net.build import TFNet
import cv2
import random
import signal

def randomColor():
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

def signal_received(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_received)

options = {
    "model": "cfg/yolo-auvsi.cfg",
    "load": "bin/tiny-yolo-voc.weights",
    "threshold": 0.1
}

tfnet = TFNet(options)

img = cv2.imread("sample_img/0000002.jpg")

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
cv2.waitKey(0)
cv2.destroyAllWindows()
