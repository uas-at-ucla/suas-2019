from pysony import SonyAPI, payload_header
import dronekit
import urllib
import os, glob
import subprocess
import sys
import time
import thread
from collections import deque

class Photographer:
    def take_pictures(self, save_directory, photos):
        wait_between_photos = 1.2

        photo_urls = deque()
        thread.start_new_thread(self.download_pictures_from_deque, (photo_urls,
            save_directory, photos, wait_between_photos ))

        while 1:
            try:
                #pixhawk = dronekit.connect(ip = "127.0.0.1:8081", baud = 57600)
                #print pixhawk.location.global_frame

                # Take pictures forever.
                camera = SonyAPI()
                camera.QX_ADDR = "http://192.168.122.1:8080"

                while 1:
                    url = camera.actTakePicture()['result'][0][0]
                    url = url.replace("\/", "/")
                    #url = url.replace("Scn", "Origin") # original
                    url = url.replace("Origin", "Scn") # preview
                    photo_urls.append(url)
                    time.sleep(wait_between_photos)

                pixhawk.close()
            except:
                print "Error accessing camera."
                time.sleep(1)

    def download_pictures_from_deque(self, photo_urls, save_directory, photos, \
            wait_between_photos):
        i = 0

        while True:
            if len(photo_urls) == 0:
                time.sleep(1 / 100.0)
                continue

            url = photo_urls.pop()
            image_path = save_directory + "/" + str(i).zfill(5)
            try:
                urllib.urlretrieve(url, image_path + ".jpg")
                photos.append(image_path)
                i = i + 1;
                print "Picture #" + str(i) + " stored at " + image_path
            except:
                print "################Picture download cut short##############"
                wait_between_photos = wait_between_photos + 0.1

                print "Increasing photo interval to " + \
                        str(wait_between_photos)
                time.sleep(1)
