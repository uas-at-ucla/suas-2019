import numpy as np
import cv2
import os
import sys
import webcolors
from matplotlib import pyplot as plt


#image_path = "/home/uas/Documents/suas_2018/src/vision/target_generator/DATA/train_images/0000000.jpg"
#image_path = "/home/uas/Documents/suas_2018/src/vision/target_generator/DATA/cropped_images/56.jpg"

'''
def load_images(): #load image(s)
	image = cv2.imread(image_path)
	if image is None:
		raise ValueError("image was not loaded")
	return image
'''

def display(name, image): #show image
	cv2.imshow(name, image)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


#def inRange(color):
        

class Color:
	def __init__(self, rlow, rhigh, glow, ghigh, blow, bhigh):
		self.rlow = rlow
		self.rhigh = rhigh
	        self.glow = glow
                self.ghigh = ghigh
                self.blow = blow
                self.bhigh = bhigh


blue = Color(0, 50, 0, 50, 155, 255)
red = Color(155, 255, 0, 50, 0, 50)
green = Color(0, 55, 110, 255, 0, 55)


colors = {
        blue: "blue",
        red: "red",
        green: "green"
}

class ColorDetector:
	def __init__(self): #initialize color arrays
		self.blue = np.array([255, 0, 0])
		self.green = np.array([0, 255, 0])
		self.red = np.array([0, 0, 255])
	
	def create_histogram(self, image): #get the color histogram of the image
		n_bins = 256
		blue = cv2.calcHist([image], [0], None, [n_bins], [0, 256])
	        green = cv2.calcHist([image], [1], None, [n_bins], [0, 256])
       		red = cv2.calcHist([image], [2], None, [n_bins], [0, 256])
		#print("blue: {}".format(np.argmax(blue)))
                #print("green: {}".format(np.argmax(green)))
                #print("red: {}".format(np.argmax(red)))
		
		'''
        	plt.subplot(221) #pyplot ish
        	plt.plot(blue, color="b")
        	plt.xlim([0, 256])
        	plt.subplot(223)
        	plt.plot(green, color="g")
        	plt.xlim([0, 256])
        	plt.subplot(224)
        	plt.plot(red, color="r")
        	plt.xlim([0, 256])
        	plt.show()
		'''
		
		return np.argmax(blue), np.argmax(green), np.argmax(red)

        
def inRange(low, high, value):
        return value <= high and value >= low


def within_buffer(val1, val2, val3):
        return abs(val1 - val2) < 20 and abs(val2 - val3) < 20 and abs(val1 - val3) < 20 


def get_color(blue, green, red):
        if within_buffer(blue, green, red):
                if blue < 60:
                        return "black"
                elif blue > 190:
                        return "white"
                else: #gray is anything in between
                        return "gray"
        for color in colors:
                if inRange(color.blow, color.bhigh, blue) and inRange(color.glow, color.ghigh, green) and inRange(color.rlow, color.rhigh, red):
                        return colors[color]

                if blue < 50 and green < 50 and red > 180:
                        return "red"

                if (blue > 150 and green < 50 and red < 50) or (red < 40 and green > 50 and blue > 120 and green < blue) or (red < 20 and green < 20 and blue > 100):
                        return "blue"
                
        return "unknown"

        
if __name__ == "__main__":
	#image = load_images()
	detector = ColorDetector()
        assert(len(sys.argv) == 2)
        
        img_path = "/home/uas/Documents/suas_2018/src/vision/target_generator/DATA/cropped_images/" + str(sys.argv[1]) + ".jpg"
        print(img_path)
        image = cv2.imread(img_path)
	blue, green, red = detector.create_histogram(image)
        print(get_color(blue, green, red))
	#print(webcolors.rgb_to_name((0, 0, 0)))
	display("test", image)
