from PIL import Image
import numpy as np
import cv2
from keras.models import load_model
import letter_seg as ls

def shape_img(path):
    img = Image.open(path)
    img = img.resize((224,224))
    img = img.reshape((224,224))
    img = np.array(img).astype(np.float32)
    img = np.expand_dims(img, axis=0)
    return img

def letter_img(path):
    img = ls.letter_seg(path)
    img = cv2.resize(img, dsize=(32, 32), interpolation=cv2.INTER_LINEAR)
    return img

def load_model(path):
    return load_model(path)

def predict(model, img):
    return np.argmax(model.predict(img))