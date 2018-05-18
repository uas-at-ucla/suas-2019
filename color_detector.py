from keras.models import Sequential
from keras.layers import Dense, Activation
from keras.utils.np_utils import to_categorical
from histogram_calculations import ColorDetector
import h5py
import numpy as np
import cv2

directory = "/home/uas/Documents/suas_2018/src/vision/target_generator/DATA/cropped_images/"
labels_path = "/home/uas/Documents/suas_2018/src/vision/target_generator/DATA/labels-5-1-2018.txt"


def load_data(): #load the labels and the rgb values corresponding to the images with the corresponding label 
	detector = ColorDetector() #create new color detector instance
	image_id = 0 
	f = open(labels_path)
	lines = f.readlines() #get the labels from the labels file
	labels = [line.strip('\n') for line in lines] #store the labels in a list
	labels = list(map(int, labels)) #convert from string to int
	while image_id <= 999:
		image_path = directory + str(image_id) + ".jpg"
		image = cv2.imread(image_path) #read the image
		blue, green, red = detector.create_histogram(image) #get the rgb values
		rgb = [red, green, blue] #store as array
		rgb = np.asarray(rgb)
		rgb_values.append(rgb) #add to rgb list
		image_id += 1
	
	rgb_values = np.asarray(rgb_values)
	labels = np.asarray(labels)
	return rgb_values, labels #return the rgb values and labels


def create_model(X, Y): #defining the model 
	model = Sequential()
	model.add(Dense(30, input_shape=(3, ), activation='sigmoid'))
	model.add(Dense(25, activation='sigmoid'))
	model.add(Dense(25, activation='sigmoid'))
	model.add(Dense(10, activation='sigmoid'))
	#model.add(Activation("softmax"))
	model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])
	hist = model.fit(X, Y, batch_size=10, epochs=1000, validation_split=0.1)
	#print(hist.history)
	return model

if __name__ == '__main__':
	rgb_values, labels = load_data()
	print(rgb_values.shape)
	labels = to_categorical(labels, num_classes=10)
	#print(labels[366])
	model = create_model(rgb_values, labels) #generate the model
	model.save("/home/uas/Documents/suas_2018/src/vision/train_model_v2.1.h5") #save the model
