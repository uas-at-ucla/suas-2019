import sys
from PIL import Image
import numpy as np
import cv2
import keras.models
sys.path.insert(0, 'segmentation')
import segmentation.letter_seg as ls

IMG_SIZE = 100

SHAPE_INDEX = [
    "Circle",
    "SemiCircle",
    "QuarterCircle",
    "Triangle",
    "Square",
    "Rectangle",
    "Trapezoid",
    "Pentagon",
    "Hexagon",
    "Heptagon",
    "Octagon",
    "Star",
    "Cross"
]

LETTERS = [
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
    'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
]

def shape_img(path):
    img = Image.open(path)
    img = img.resize((IMG_SIZE, IMG_SIZE))
    #img = img.reshape((IMG_SIZE, IMG_SIZE))
    img = np.array(img).astype(np.float32)
    img = np.expand_dims(img, axis=0)
    return img

def letter_img(path):
    img = ls.letter_seg(path)
    img = cv2.resize(img, dsize=(32, 32), interpolation=cv2.INTER_LINEAR)
    img = np.expand_dims(img, axis=2) 
    img = np.expand_dims(img, axis=0)

    im = img.astype('uint8')[0]
    im[im > 0] = 255
    im = np.repeat(im, 3, axis=2);
    im = Image.fromarray(im)
    im.save('seg/img' + path[-9:])
    # np.savetxt('seg/img' + path[-9:] + '.csv', img[0,:,:,0])
    return img

def load_model(path):
    return keras.models.load_model(path)

def predict(model, img):
    return np.argmax(model.predict(img))

def predict_shape(model, img):
    return SHAPE_INDEX[predict(model, img).argmax()]

def predict_letter(model, img):
    return LETTERS[predict(model, img).argmax()]

