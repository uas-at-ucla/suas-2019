import sys
import random, string
import glob, os
from PIL import Image
from PIL import ImageFilter
import numpy as np
import math

#TODO: Two folders: images, txt files: one w vertices of images

class ImageSimulator:
    def __init__(self):
        self.targets = dict()
        self.fields = list()

        os.chdir(os.path.dirname(os.path.realpath(__file__)))

        os.chdir("targets")
        for file_name in glob.glob("*.png"):
            name = file_name.replace('.png', '')
            self.targets[name] = Image.open(file_name).convert("RGBA")

        os.chdir("../fields")
        for file_name in glob.glob("*.jpg"):
            name = file_name.replace('.jpg', '')
            self.fields.append(Image.open(file_name).convert("RGBA"))

        background = self.fields[0]
        backgroundWidth,backgroundHeight = background.size

        for i in self.targets:

            #make background copy, rotate to random angle
            backgroundCopy = background.copy()
            angle = random.randint(0,359)
            target = self.targets[i].rotate(angle)

            #scale target size
            scale = (random.randint(1,15))*0.1
            targetWidth,targetHeight = target.size
            targetWidth = int(targetWidth*scale)
            targetHeight = int(targetHeight*scale)

            targetSize = targetWidth,targetHeight
            target = target.resize((targetWidth,targetHeight), Image.ANTIALIAS)
            margin = 0
            if(targetWidth > targetHeight):
                margin = targetWidth
            else:
                margin = targetHeight

            #randomized location for target
            xPos = random.randint(0,backgroundWidth-margin)
            yPos = random.randint(0,backgroundHeight-margin)

            #set random color
            randomR = random.randint(0,255)
            randomG = random.randint(0,255)
            randomB = random.randint(0,255)

            data = np.array(target)
            r1,g1,b1 = 255,255,255
            r2,g2,b2 = randomR,randomG,randomB

            red, green, blue = data[:,:,0], data[:,:,1], data[:,:,2]
            mask = (red <= r1) & (green <= g1) & (blue <= b1)
            data[:,:,:3][mask] = [r2, g2, b2]

            #blur target
            blur = random.randint(0,10)
            target = Image.fromarray(data)
            target = target.filter(ImageFilter.GaussianBlur(blur))

            #paste target on field and display
            backgroundCopy.paste(target, (xPos, yPos), target)
            backgroundCopy.show()


    def random_filename(self, length):
       return ''.join(random.choice(string.lowercase) for i in range(length))

def main():
    if len(sys.argv) is not 1:
        print("Expected an output folder to store synthesized images in.")

    image_simulator = ImageSimulator()

if __name__ == "__main__":
    main()
