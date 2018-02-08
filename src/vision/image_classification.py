from keras.models import Sequential, Model
from keras.layers import Permute, Conv2D, MaxPooling2D, Flatten, Activation, Dropout
from PIL import Image
from keras import backend as K
import numpy as np

def model(type='letter'):
    types = {'letter': 26, 'shape':8}
    
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
    model.add( Dropout(0.5) )
    model.add(Conv2D(4096, kernel_size=(1,1), activation='relu', name='fc7'))
    model.add( Dropout(0.5) )
    model.add(Conv2D(types[type], kernel_size=(1, 1), activation='relu', name='fc8'))
    model.add(Flatten())
    model.add(Activation('softmax'))
    
    return model

def process_img(addr):
    img = Image.open(addr).convert('RGB')
    img = Image.reshape(224,224)
    img = np.array(img).astype(np.float32)
    img = np.expand_dims(img, axis=0)
    
    return img

def train(model, x_train, y_train, x_test, y_test, x_val, y_val):
    model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
    model.fit(x=x_train, y=y_train, validation_data=(x_val, y_val) batch_size=1, epochs=9, verbose=1)
    model.evaluate(x=x_test, y=y_test, verbose=1)
    
    return model

def predict(model, img):
    return model.predict(img)