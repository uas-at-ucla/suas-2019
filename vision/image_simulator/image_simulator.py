import sys
import random, string
import glob, os
from PIL import Image
import numpy as np

#TODO: scale, blur, color, (boundary conditions)


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
        width,height = background.size

        for i in self.targets:

            #random position, angle, color values

            randomR = random.randint(0,255)
            randomG = random.randint(0,255)
            randomB = random.randint(0,255)
            xPos = random.randint(0,width)
            yPos = random.randint(0,height)
            angle = random.randint(0,359)

            backgroundCopy = background.copy()
            target = self.targets[i].rotate(angle)

            data = np.array(target)
            r1,g1,b1 = 0,0,0
            r2,g2,b2 = randomR,randomG,randomB

            red, green, blue = data[:,:,0], data[:,:,1], data[:,:,2]
            mask = (red == r1) & (green == g1) & (blue == b1)
            data[:,:,:3][mask] = [r2, g2, b2]

            target = Image.fromarray(data)

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
