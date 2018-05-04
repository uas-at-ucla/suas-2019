import cv2
import numpy as np
import os
from collections import deque
from xml.dom import minidom
import lxml.etree


targets_path = "/home/uas/Documents/suas_2018/src/vision/target_generator/DATA/train_images/" #where the full image is located
output_directory = "/home/uas/Documents/suas_2018/src/vision/target_generator/DATA/cropped_images/" #where to write the cropped images

if not os.path.isdir(output_directory): #create the output directory if it doesn't exist
	os.mkdir(output_directory)

files = os.listdir(targets_path) #get the images and labels from the training directory
files.sort() 

def read_images():
	images = deque() #for storing the images in the directory 
	info = deque() #for storing the lables in the directory
	for name in files:
		path = targets_path + name #file path
		if ".jpg" in name: #if is a jpg file
			image = cv2.imread(path) #read the image
			if image is None:
				raise ValueError("image was not loaded properly") #throw exception if image was not loaded (idk how to really throw exceptions lol)
			images.append(image) #else store in container
		elif ".xml" in name: #if is a label
			info.append(name)	

	print("finished reading all images")
	return images, info


if __name__ == "__main__":
	images, labels = read_images()	

	assert(len(images) == len(labels)) #make sure that all images have a label

	print("extracting targets from image")

	image_number = 0
	for i in range(len(labels)):
		doc = lxml.etree.parse(targets_path + "/" + labels[i]) #parse xml file
		xmin = doc.find('object/bndbox/xmin') #read the bounding box data
		ymin = doc.find('object/bndbox/ymin')
		xmax = doc.find('object/bndbox/xmax')
		ymax = doc.find('object/bndbox/ymax')

		image = images[i] 
		cropped = image[int(ymin.text): int(ymax.text), int(xmin.text): int(xmax.text)] #extract ROI
		image_name = output_directory + str(image_number) + ".jpg"

		cv2.imwrite(image_name, cropped) #save image 
		
		image_number += 1

	print("finished writing cropped images to {}".format(output_directory))
