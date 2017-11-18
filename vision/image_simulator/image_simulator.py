import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
os.chdir(os.path.dirname(os.path.realpath(__file__)))

import sys
import random, string
import glob, os
from PIL import Image
from PIL import ImageFilter
import numpy as np
import math
import threading
import signal
import time

killed = False

#TODO: Two folders: images, txt files: one w vertices of images
class ImageSimulator:
    def __init__(self):
        self.targets = dict()
        self.targets_class = dict()
        self.fields = list()

        os.chdir("targets")
        target_index = 0
        for file_name in glob.glob("*.png"):
            name = file_name.replace('.png', '')
            self.targets[name] = Image.open(file_name).convert("RGBA")
            self.targets_class[name] = target_index
            target_index += 1

        os.chdir("../fields")
        for file_name in glob.glob("*.jpg"):
            name = file_name.replace('.jpg', '')
            self.fields.append(Image.open(file_name).convert("RGBA"))
        os.chdir("..")

    def generate_image(self, shape, i):
        background = self.fields[0]
        backgroundWidth, backgroundHeight = background.size

        scale = 0.15
        backgroundWidth = int(float(backgroundWidth) * scale)
        backgroundHeight = int(float(backgroundHeight) * scale)
        background = background.resize(\
                (backgroundWidth, backgroundHeight), \
                Image.ANTIALIAS)

        print("Image width: " + str(backgroundWidth) \
                + " image heigth: " + str(backgroundHeight))

        # Make background copy, rotate to random angle
        backgroundCopy = background.copy()
        #angle = random.randint(0, 359)
        angle = 0
        target = self.targets[shape].rotate(angle)

        # Scale target size
        scale = (random.randint(12, 15)) * 0.1
        targetWidth, targetHeight = target.size
        targetWidth = 100 #int(targetWidth * scale)
        targetHeight = 100 #int(targetHeight * scale)

        targetSize = targetWidth, targetHeight
        target = target.resize((targetWidth, targetHeight), Image.ANTIALIAS)
        margin = 0
        if(targetWidth > targetHeight):
            margin = targetWidth
        else:
            margin = targetHeight

        # Randomized location for target
        xPos = random.randint(0, backgroundWidth - margin)
        yPos = random.randint(0, backgroundHeight - margin)

        # Set random color
        randomR = random.randint(0 ,255)
        randomG = random.randint(0, 255)
        randomB = random.randint(0, 255)

        data = np.array(target)
        r1, g1, b1 = 255, 255, 255
        r2, g2, b2 = randomR, randomG, randomB

        red, green, blue = data[:, :, 0], data[:, :, 1], data[:, :, 2]
        mask = (red <= r1) & (green <= g1) & (blue <= b1)
        data[:, :, :3][mask] = [r2, g2, b2]

        # Blur target
        blur = random.randint(2, 4)
        target = Image.fromarray(data)
        target = target.filter(ImageFilter.GaussianBlur(blur))

        # Paste target on field
        backgroundCopy.paste(target, (xPos, yPos), target)
        backgroundCopy = backgroundCopy.convert("RGB")

        # Track where the object was placed.
        filename = shape + "_" + str(i).zfill(6)

        # Create directory structure.
        self.create_directory("output")
        self.create_directory("output/images")

        self.create_directory("output/labels")

        # Write image to file
        output_image_file = "output/images/" + filename + ".jpg"
        backgroundCopy.save(output_image_file)

        # Write label to file
        output_label_file = "output/labels/" + filename + ".xml"
        text_file = open(output_label_file, "w")
        target_label = \
"<annotation>\n" \
"    <filename>" + filename + ".jpg</filename>\n" \
"   <size>\n" \
"       <width>" + str(int(backgroundWidth)) + "</width>\n" \
"       <height>" + str(int(backgroundHeight)) + "</height>\n" \
"   </size>\n" \
"   <object>\n" \
"       <name>" + shape + "</name>\n" \
"       <bndbox>\n" \
"           <xmin>" + str(xPos) + "</xmin>\n" \
"           <ymin>" + str(yPos) + "</ymin>\n" \
"           <xmax>" + str(xPos + targetWidth) + "</xmax>\n" \
"           <ymax>" + str(yPos + targetHeight) + "</ymax>\n" \
"       </bndbox>\n" \
"   </object>\n" \
"</annotation>"
        text_file.write(target_label)
        text_file.close()

    def random_filename(self, length):
       return ''.join(random.choice(string.lowercase) for i in range(length))

    def create_directory(self, directory):
        try:
            os.stat(directory)
        except:
            os.mkdir(directory)

def generate_image_thread(image_simulator, total_images, thread_num, \
        total_threads, threads_finished):
    i = thread_num

    global killed

    while i < total_images:
        for shape in image_simulator.targets:
            if killed:
                return

            image_simulator.generate_image(shape, i)

        print("Thread " + str(thread_num) + " on image " + str(i))

        i += total_threads

    threads_finished[thread_num] = True

def signal_received(signal, frame):
    print("SIGNAL RECEIVED")
    global killed
    killed = True
    sys.exit(1)

def main():
    if len(sys.argv) is not 1:
        print("Expected an output folder to store synthesized images in.")

    signal.signal(signal.SIGINT, signal_received)

    image_simulator = ImageSimulator()

    total_images = 100
    total_threads = 8

    threads = list()
    threads_finished = list()
    for i in range(0, total_threads):
        threads_finished.append(False)

    for i in range(0, total_threads):
        new_thread = threading.Thread(target = generate_image_thread, \
            args = (image_simulator, total_images, i, total_threads,
                threads_finished))
        threads.append(new_thread)
        new_thread.start()

    for cur_thread in range(0, total_threads):
        while True:
            if threads_finished[cur_thread]:
                print("Thread " + str(cur_thread) + " completed!")
                break
            else:
                time.sleep(0.5)

    print("Finished generating " + str(total_images) + " images!")

    killed = True

if __name__ == "__main__":
    main()
