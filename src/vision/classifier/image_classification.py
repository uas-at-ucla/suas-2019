from keras.models import Sequential, Model
from keras.layers import Dense, Permute, Conv2D, MaxPooling2D, Flatten, Activation, Dropout
from keras.utils import to_categorical
from PIL import Image
from keras import backend as K
from keras.optimizers import SGD
import numpy as np
import tables

def letter_model():
    K.set_image_data_format( 'channels_last' )

    model = Sequential()

    model.add(Permute((1,2, 3), input_shape=(32, 32, 1)))

    model.add(Conv2D(32, kernel_size=(5,5), activation='relu', kernel_initializer='TruncatedNormal', name="conv1"))
    model.add(MaxPooling2D((2,2), strides=(2,2)))
    model.add(Dropout(.25))

    model.add(Conv2D(64, kernel_size=(5,5), activation='relu', kernel_initializer='TruncatedNormal', name="conv2"))
    model.add(MaxPooling2D((2,2), strides=(2,2)))
    model.add(Dropout(.25))

    model.add(Flatten())
    model.add(Dense(1600, activation='relu', name='fc3'))
    model.add(Dense(26, activation='softmax', name='fc4'))
    
    return model

def shape_model():
    K.set_image_data_format( 'channels_last' )
        
    model = Sequential()

    model.add(Permute((1,2,3), input_shape=(224, 224, 3)))

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

    model.add(Conv2D(512, kernel_size=(3,3), padding='same', activation='relu', name="conv5_1"))
    model.add(Conv2D(512, kernel_size=(3,3), padding='same', activation='relu', name="conv5_2"))
    model.add(Conv2D(512, kernel_size=(3,3), padding='same', activation='relu', name="conv5_3"))
    model.add(MaxPooling2D((2,2), strides=(2,2)))
    
    model.add(Conv2D(4096, kernel_size=(7,7), activation='relu', name='fc6'))
    model.add(Conv2D(4096, kernel_size=(1,1), activation='relu', name='fc7'))
    model.add(Conv2D(11, kernel_size=(1,1),  name='fc8'))
    model.add(Flatten())
    model.add( Activation('softmax') )
    
    return model

def process_data(data_path, type=""):
    if type == "":
        raise ValueError('Please input the type of data (shape or letter)')
    
    label_size = {"shape": 11, "letter": 26}
    
    f = tables.open_file(data_path, mode='r')

    x_train = f.root.train_input[()]
    y_train = f.root.train_labels[()]

    x_test = f.root.test_input[()]
    y_test = f.root.test_labels[()]

    f.close()

    y_train = to_categorical(y_train, label_size[type])
    y_test = to_categorical(y_test, label_size[type])

    return x_train, y_train, x_test, y_test


def train(model, x_train, y_train, x_test, y_test, nEpochs=10, nBatch=32, model_name="model"):
    
    sgd = SGD(lr=0.0001)
    model.compile(optimizer=sgd, loss='categorical_crossentropy', metrics=['accuracy'])
    model.fit(x=x_train, y=y_train, batch_size=nBatch, epochs=nEpochs, verbose=1)
    model.evaluate(x=x_test, y=y_test, verbose=1)
    model.save(model_name + '.hdf5')
    return model


def process_letter(addr):
    img = Image.open(addr).convert('L')
    img = img.reshape((32,32))
    img = np.array(img).astype(np.float32)
    img = np.expand_dims(img, axis=0)
    img = np.expand_dims(img, axis=3)
    
    return img


def process_shape(addr):
    img = Image.open(addr).convert('L')
    img = img.reshape((224,224))
    img = np.array(img).astype(np.float32)
    img = np.expand_dims(img, axis=0)

    
def predict(model, img):
    
    return model.predict(img)