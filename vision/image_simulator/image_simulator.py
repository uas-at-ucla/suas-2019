import sys
import random, string
import glob, os
from PIL import Image

class ImageSimulator:
    def __init__(self):
        self.targets = dict()
        self.fields = list()

        os.chdir(os.path.dirname(os.path.realpath(__file__)))

        os.chdir("targets")
        for file_name in glob.glob("*.png"):
            name = file_name.replace('.png', '')
            self.targets[name] = Image.open(file_name)

        os.chdir("../fields")
        for file_name in glob.glob("*.jpg"):
            name = file_name.replace('.jpg', '')
            self.fields.append(Image.open(file_name))

    def random_filename(self, length):
       return ''.join(random.choice(string.lowercase) for i in range(length))

def main():
    if len(sys.argv) is not 1:
        print("Expected an output folder to store synthesized images in.")

    image_simulator = ImageSimulator()

if __name__ == "__main__":
    main()
