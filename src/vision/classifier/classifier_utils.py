import sys
from PIL import Image
import numpy as np
import cv2
import keras.models

import letter_seg as ls
from constants import *

def shape_img(path):
    img = Image.open(path)
    img = img.resize((IMG_SIZE, IMG_SIZE))
    #img = img.reshape((IMG_SIZE, IMG_SIZE))
    img = np.array(img).astype(np.float32)
    img = np.expand_dims(img, axis=0)
    return img

def letter_img(path):
    img = ls.letter_seg(path)
    # img = cv2.resize(img, dsize=(32, 32), interpolation=cv2.INTER_LINEAR)
    img = np.expand_dims(img, axis=2) 
    img = np.expand_dims(img, axis=0)

    im = img.astype('uint8')[0]
    im[im > 0] = 255
    im = np.repeat(im, 3, axis=2);
    im = Image.fromarray(im)
    im.save('seg/img' + path[-9:])
    # np.savetxt('seg/img' + path[-9:] + '.csv', img[0,:,:,0])
    return img

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
        img_func[feature](path)[0,:]
        # x_train[i] = img_func[feature](path)[0,:]
    y_train = to_categorical(y_train, label_size[feature])
    return x_train, y_train

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

