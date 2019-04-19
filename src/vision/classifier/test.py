import os
import segmentation.letter_seg as ls
import vision_classifier as vc 
import image_classification as ic

IMG_SIZE = 224
N_TRAIN = 10
N_TEST = 2

print(os.getcwd())

train_dir = '../targets/output/train/'
test_dir = '../targets/output/test/'

def get_train_and_test(feature, n_train, n_test):
    (x_train, y_train) = ic.process_data(train_dir, n_train, feature)
    (x_test, y_test) = ic.process_data(test_dir, n_test, feature)
    return x_train, y_train, x_test, y_test

#letter_seg('../targets/output/train/00.jpg')
img = vc.shape_img('../targets/output/train/000.jpg')
print(img.shape)

shape_model = ic.shape_model()
letter_model = ic.letter_model()
print(shape_model)
print(letter_model)

# Train on shapes

(x_train, y_train, x_test, y_test) = get_train_and_test('shape', N_TRAIN, N_TEST)
print(x_train.shape, y_train.shape, x_test.shape, y_test.shape)

shape_model = ic.train(shape_model, x_train, y_train, x_test, y_test, 
                       model_name='shape_model')
shape_model.evaluate(x=x_test, y=y_test)

# img = process_letter('../targets/output/train/00.jpg')
# print(img.shape)

# Train on letters

(x_train, y_train, x_test, y_test) = get_train_and_test('letter', N_TRAIN, N_TEST)
print(x_train.shape, y_train.shape, x_test.shape, y_test.shape)

letter_model = ic.train(letter_model, x_train, y_train, x_test, y_test, 
                       model_name='letter_model')
letter_model.evaluate(x=x_test, y=y_test)


# TODO: save model

# shape_model = vc.load_model('../models/shape/shape_model.h5')
# letter_model = vc.load_model('../models/letter/letter_model.hdf5')

# old_dir = '../targets/output/old'
# xs_old, ys_old = ic.process_data(old_dir, 1000, 'shape', True)
# xl_old, yl_old = ic.process_data(old_dir, 1000, 'letter', True)
# print(xs_old.shape, ys_old.shape, xl_old.shape, yl_old.shape)

# shape_model.evaluate(x=xs_old, y=ys_old)
# letter_model.evaluate(x=xl_old, y=yl_old)

# i = 1
# path = '../targets/output/old/%03d.jpg'%i
# print(predict(shape_model, vc.shape_img(path)))
# print('predicted: ', vc.predict_shape(shape_model, vc.shape_img(path)))
# print('true: ', vc.SHAPE_INDEX[ys_old[i].argmax()])
# print(predict(letter_model, vc.letter_img(path)))
# print('predicted: ', vc.predict_letter(letter_model, vc.letter_img(path)))
# # print('true: ', chr(yl_old[i].argmax() + 65))
# print('true: ', LETTERS[predict(model, img).argmax()])

# TODO: automate, set up to do some training

