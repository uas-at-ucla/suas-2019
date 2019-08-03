import os
from PIL import Image
import numpy as np

from keras.models import Sequential, Model
from keras.layers import Dense, Permute, Conv2D, MaxPooling2D, Flatten, Activation, Dropout
from keras.callbacks import ModelCheckpoint
from keras.utils import to_categorical
from keras import backend as K
from keras.optimizers import SGD

from constants import *

def letter_model():
    K.set_image_data_format( 'channels_last' )
    model = Sequential()
    model.add(Permute((1,2,3), input_shape=(32, 32, 1)))

    model.add(Conv2D(32, kernel_size=(5,5), activation='relu', kernel_initializer='TruncatedNormal', name="conv1"))
    model.add(MaxPooling2D((2,2), strides=(2,2)))
    model.add(Dropout(.25))

    model.add(Conv2D(64, kernel_size=(5,5), activation='relu', kernel_initializer='TruncatedNormal', name="conv2"))
    model.add(MaxPooling2D((2,2), strides=(2,2)))
    model.add(Dropout(.25))

    model.add(Flatten())
    model.add(Dense(1600, activation='relu', name='fc3'))
    model.add(Dense(N_LETTERS, activation='softmax', name='fc4'))
    return model

def shape_model(n_outputs=N_SHAPES):
    K.set_image_data_format( 'channels_last' )
    model = Sequential()
    model.add(Permute((1,2,3), input_shape=(IMG_SIZE, IMG_SIZE, 3)))

    model.add(Conv2D(64, kernel_size=(3,3), padding='same', activation='relu', name="conv1_1"))
    model.add(Conv2D(64, kernel_size=(3,3), padding='same', activation='relu', name="conv1_2"))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(Conv2D(128, kernel_size=(3,3), padding='same', activation='relu', name="conv2_1"))
    model.add(Conv2D(128, kernel_size=(3,3), padding='same', activation='relu', name="conv2_2"))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(Conv2D(256, kernel_size=(3,3), padding='same', activation='relu', name="conv3_1"))
    model.add(Conv2D(256, kernel_size=(3,3), padding='same', activation='relu', name="conv3_2"))
    model.add(Conv2D(256, kernel_size=(3,3), padding='same', activation='relu', name="conv3_3"))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(Conv2D(512, kernel_size=(3,3), padding='same', activation='relu', name="conv4_1"))
    model.add(Conv2D(512, kernel_size=(3,3), padding='same', activation='relu', name="conv4_2"))
    model.add(Conv2D(512, kernel_size=(3,3), padding='same', activation='relu', name="conv4_3"))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    # model.add(Conv2D(512, kernel_size=(3,3), padding='same', activation='relu', name="conv5_1"))
    # model.add(Conv2D(512, kernel_size=(3,3), padding='same', activation='relu', name="conv5_2"))
    # model.add(Conv2D(512, kernel_size=(3,3), padding='same', activation='relu', name="conv5_3"))
    # model.add(MaxPooling2D((2,2), strides=(2,2)))

    width = IMG_SIZE//16
    model.add(Conv2D(4096, kernel_size=(width, width), activation='relu', name='fc6'))
    model.add(Conv2D(4096, kernel_size=(1,1), activation='relu', name='fc7'))
    model.add(Conv2D(n_outputs, kernel_size=(1,1),  name='fc8'))
    model.add(Flatten())
    model.add( Activation('softmax') )
    return model

def alt_letter_model():
    return shape_model(N_LETTERS)

def train(model, x_train, y_train, x_test, y_test, nEpochs=10, nBatch=32, model_name="model"):
    sgd = SGD(lr=0.00001)
    model.compile(optimizer=sgd, loss='categorical_crossentropy', metrics=['accuracy'])
    os.makedirs('models', exist_ok=True)
    checkpoint = ModelCheckpoint('models/' + model_name + '{epoch:02d}.hdf5')
    model.fit(x=x_train, y=y_train, batch_size=nBatch, epochs=nEpochs, verbose=1,
              callbacks=[checkpoint])
    model.evaluate(x=x_test, y=y_test, verbose=1)
    model.save(model_name + '.hdf5')
    return model

def load_model(path):
    return keras.models.load_model(path)

def predict(model, img):
    return np.argmax(model.predict(img))

def predict_shape(model, img):
    return SHAPES[predict(model, img).argmax()]

def predict_letter(model, img):
    return LETTERS[predict(model, img).argmax()]

