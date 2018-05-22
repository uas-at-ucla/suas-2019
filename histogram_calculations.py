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



class ColorDetector:
	def __init__(self): #initialize arrays
		self.blue = np.array([255, 0, 0])
		self.green = np.array([0, 255, 0])
		self.red = np.array([0, 0, 255])
	
	def create_histogram(self, image): #get the color histogram of the image
		n_bins = 256
		blue = cv2.calcHist([image], [0], None, [n_bins], [0, 256])
	        green = cv2.calcHist([image], [1], None, [n_bins], [0, 256])
       		red = cv2.calcHist([image], [2], None, [n_bins], [0, 256])
		
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

        
        
if __name__ == "__main__":
	detector = ColorDetector()
        assert(len(sys.argv) == 2)
        
        img_path = "/home/uas/Documents/suas_2018/src/vision/target_generator/DATA/cropped_images/" + str(sys.argv[1]) + ".jpg"
        print(img_path)
        image = cv2.imread(img_path)
	blue, green, red = detector.create_histogram(image)
        print(get_color(blue, green, red))
	display("test", image)
