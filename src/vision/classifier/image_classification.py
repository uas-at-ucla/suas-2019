import os
from keras.models import Sequential, Model
from keras.layers import Dense, Permute, Conv2D, MaxPooling2D, Flatten, Activation, Dropout
from keras.callbacks import ModelCheckpoint
from keras.utils import to_categorical
from PIL import Image
from keras import backend as K
from keras.optimizers import SGD
import numpy as np

import vision_classifier as vc

N_LETTERS = 36 # include numbers
N_SHAPES = 13
IMG_SIZE = 100

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

# def old_process_data(data_path, type=""):
#     if type == "":
#         raise ValueError('Please input the type of data (shape or letter)')
#     label_size = {"shape": 11, "letter": 26}
#     f = tables.open_file(data_path, mode='r')
#     x_train = f.root.train_input[()]
#     y_train = f.root.train_labels[()]
#     x_test = f.root.test_input[()]
#     y_test = f.root.test_labels[()]
#     f.close()
#     y_train = to_categorical(y_train, label_size[type])
#     y_test = to_categorical(y_test, label_size[type])
#     return x_train, y_train, x_test, y_test

def process_data(data_dir, n_images, feature="", old=False):
    if feature == "":
        raise ValueError('Please input the type of data (shape or letter)')
    label_size = {"shape": N_SHAPES, "letter": N_LETTERS}
    if old:
        label_size = {"shape": 11, "letter": 26}
    img_size = {"shape": (IMG_SIZE, IMG_SIZE, 3), "letter": (32, 32, 1)}
    label_col = {"shape": 5, "letter": 6}
    img_func = {"shape": vc.shape_img, "letter": vc.letter_img}
    # Load data from data_dir
    x_train = np.zeros((n_images,) + img_size[feature])
    path = os.path.join(data_dir, 'labels.csv')
    labels = np.loadtxt(path, delimiter=',', skiprows=1)
    y_train = labels[0:n_images, label_col[feature]]
    for i in range(n_images):
        path = os.path.join(data_dir, '%05d.jpg'%i)
        x_train[i] = img_func[feature](path)[0,:]
    y_train = to_categorical(y_train, label_size[feature])
    return x_train, y_train

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

# def process_letter(addr):
#     img = Image.open(addr).convert('L')
#     img = img.reshape((32,32))
#     img = np.array(img).astype(np.float32)
#     img = np.expand_dims(img, axis=0)
#     img = np.expand_dims(img, axis=3)
#     return img

# def process_shape(addr):
#     img = Image.open(addr).convert('L')
#     img = img.reshape((IMG_SIZE,IMG_SIZE))
#     img = np.array(img).astype(np.float32)
#     img = np.expand_dims(img, axis=0)

