import sys
from PIL import Image
import numpy as np
import cv2
import keras.models
sys.path.insert(0, 'segmentation')
import segmentation.letter_seg as ls

SHAPE_INDEX = [
    'circle', 'cross', 'heptagon', 'octagon', 'pentagon', 'quarter_circle',
    'rectangle', 'semicircle', 'star', 'trapezoid', 'triangle'
]


def shape_img(path):
    img = Image.open(path)
    img = img.resize((224, 224))
    #img = img.reshape((224, 224))
    img = np.array(img).astype(np.float32)
    img = np.expand_dims(img, axis=0)
    return img


def letter_img(path):
    img = ls.letter_seg(path)
    img = cv2.resize(img, dsize=(32, 32), interpolation=cv2.INTER_LINEAR)
    img = np.expand_dims(img, axis=2) 
    img = np.expand_dims(img, axis=0)
    return img


def load_model(path):
    return keras.models.load_model(path)


def predict(model, img):
    return np.argmax(model.predict(img))


def predict_shape(model, img):
    return SHAPE_INDEX[predict(model, img)]


def predict_letter(model, img):
    return chr(predict(model, img) + 65)
