from random import shuffle
import glob
import numpy as np
import tables
from PIL import Image

paths = ['DATA/letter_A/*.jpg', 'DATA/letter_B/*.jpg', 'DATA/letter_C/*.jpg', 
         'DATA/letter_D/*.jpg', 'DATA/letter_E/*.jpg', 'DATA/letter_F/*.jpg',
         'DATA/letter_G/*.jpg', 'DATA/letter_H/*.jpg', 'DATA/letter_I/*.jpg',
         'DATA/letter_J/*.jpg', 'DATA/letter_K/*.jpg', 'DATA/letter_L/*.jpg',
         'DATA/letter_M/*.jpg', 'DATA/letter_N/*.jpg', 'DATA/letter_O/*.jpg',
         'DATA/letter_P/*.jpg', 'DATA/letter_Q/*.jpg', 'DATA/letter_R/*.jpg',
         'DATA/letter_S/*.jpg', 'DATA/letter_T/*.jpg', 'DATA/letter_U/*.jpg',
         'DATA/letter_V/*.jpg', 'DATA/letter_W/*.jpg', 'DATA/letter_X/*.jpg',
         'DATA/letter_Y/*.jpg', 'DATA/letter_Z/*.jpg']

addrs = []
labels = []

i = 0
for path in paths:
    letter_addrs = glob.glob(path)
    letter_labels = [i for x in range(len(letter_addrs))]
    
    addrs.extend(letter_addrs)
    labels.extend(letter_labels)
    i += 1

c = list(zip(addrs, labels))
shuffle(c)

addrs, labels = zip(*c)

train_input = addrs[0:int(0.9 * len(addrs))]
train_labels = labels[0:int(0.9 * len(labels))]

test_input = addrs[int(0.9 * len(addrs)):]
test_labels = labels[int(0.9 * len(labels)):]

i_shape = (0, 32, 32)

dt = tables.Float32Atom()

f = tables.open_file('data.hdf5', mode='w')

train_input_arr = f.create_earray(f.root, 'train_input', dt, shape=i_shape)
test_input_arr = f.create_earray(f.root, 'test_input', dt, shape=i_shape)

train_labels_arr = f.create_array(f.root, 'train_labels', train_labels)
test_labels_arr = f.create_array(f.root, 'test_labels', test_labels)


for i in range(len(train_input)):
    addr = train_input[i]
    
    img = Image.open(addr).convert('L')
    img = img.resize((32, 32))
    img = np.array(img).astype(np.float32)
    
    train_input_arr.append(img[None])

for i in range(len(test_input)):
    addr = test_input[i]
    
    img = Image.open(addr).convert('L')
    img = img.resize((32, 32))
    img = np.array(img).astype(np.float32)
    
    test_input_arr.append(img[None])

f.close()