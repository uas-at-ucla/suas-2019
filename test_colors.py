from keras.models import load_model
from histogram_calculations import ColorDetector
from argparse import ArgumentParser
import numpy as np #todo: use argparse to print the array of predictions
import cv2
import h5py
import sys


color_dict = {
	1: "blue",
	2: "red",
	3: "green",
	4: "black",
	5: "gray",
	6: "white",
	7: "yellow",
	8: "purple",
	9: "brown"
}

def softmax(predictions):
	print(predictions.shape)
	return np.exp(predictions)/(sum(np.exp(predictions)))

def main(show):
	model = load_model("/home/uas/Documents/suas_2018/src/vision/train_model_v2.1.h5")
	detector = ColorDetector()
	image_id = None
	if len(sys.argv) == 3:
		image_id = sys.argv[2]
	elif len(sys.argv) == 2:
		image_id = sys.argv[1]
	else:
		raise ValueError("incorrect number of arguments passed")
	
	image_path = "/home/uas/Documents/suas_2018/src/vision/target_generator/DATA/cropped_images/" + image_id + ".jpg"
	image = cv2.imread(image_path)
	b, g, r = detector.create_histogram(image)
	rgb = [r, g, b]
	values = []
	rgb = np.asarray(rgb)
	values.append(rgb)
	values = np.asarray(values)
	prediction = model.predict(values)
	result = np.argmax(prediction)	
	print(color_dict[result])
	if show:
		print((prediction))	

if __name__ == '__main__':
	parser = ArgumentParser()
	parser.add_argument("-s", "--show", help="display array of predictions for the given image id", action="store_true", dest="show")
	args = parser.parse_args(["-s"])
	if len(sys.argv) == 2:
		args.show = False
	main(args.show)
	
