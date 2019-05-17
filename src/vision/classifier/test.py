import os
import segmentation.letter_seg as ls
import vision_classifier as vc 
import image_classification as ic

IMG_SIZE = 100
N_TRAIN = 20
N_TEST = 1

print(os.getcwd())

train_dir = '../targets/output/train2/'
test_dir = '../targets/output/test2/'

def get_train_and_test(feature, n_train, n_test):
    (x_train, y_train) = ic.process_data(train_dir, n_train, feature)
    (x_test, y_test) = ic.process_data(test_dir, n_test, feature)
    return x_train, y_train, x_test, y_test

def train_shape_model(nEpochs=10):
    #letter_seg('../targets/output/train/00.jpg')
    #img = vc.shape_img('../targets/output/train/000.jpg')
    #print(img.shape)
    shape_model = ic.shape_model()
    print(shape_model)
    (x_train, y_train, x_test, y_test) = get_train_and_test('shape', N_TRAIN, N_TEST)
    print(x_train.shape, y_train.shape, x_test.shape, y_test.shape)

    shape_model = ic.train(shape_model, x_train, y_train, x_test, y_test, 
                           model_name='shape_model', nEpochs=nEpochs)
    print(shape_model.evaluate(x=x_test, y=y_test))

def train_letter_model(nEpochs=10):
    letter_model = ic.letter_model()
    print(letter_model)
    # img = process_letter('../targets/output/train/00.jpg')
    # print(img.shape)
    (x_train, y_train, x_test, y_test) = get_train_and_test('letter', N_TRAIN, N_TEST)
    print(x_train.shape, y_train.shape, x_test.shape, y_test.shape)

    letter_model = ic.train(letter_model, x_train, y_train, x_test, y_test, 
                            model_name='letter_model', nEpochs=nEpochs)
    print(letter_model.evaluate(x=x_test, y=y_test))

def train_models(nEpochs=10): 
    train_shape_model(nEpochs)
    train_letter_model(nEpochs)

def eval_shape_model(i=1):
    # shape_model = vc.load_model('../models/shape/shape_model.h5')
    shape_model = vc.load_model('shape_model.hdf5')
    # old_dir = '../targets/output/old'
    # xs_old, ys_old = ic.process_data(old_dir, 1000, 'shape', True)
    # print(xs_old.shape, ys_old.shape)
    x_shape, y_shape = ic.process_data(test_dir, 1000, 'shape')
    print(x_shape.shape, y_shape.shape)
    print(shape_model.metrics_names)
    print(shape_model.evaluate(x=x_shape, y=y_shape))
    # path = '../targets/output/old/%03d.jpg'%i
    path = '../targets/output/test2/%05d.jpg'%i
    print(vc.predict(shape_model, vc.shape_img(path)))
    print('predicted: ', vc.predict_shape(shape_model, vc.shape_img(path)))
    print('true: ', vc.SHAPE_INDEX[y_shape[i].argmax()])

def eval_letter_model(i=1):
    # letter_model = vc.load_model('../models/letter/letter_model.hdf5')
    letter_model = vc.load_model('letter_model.hdf5')
    # old_dir = '../targets/output/old'
    # xl_old, yl_old = ic.process_data(old_dir, 1000, 'letter', True)
    # print(xl_old.shape, yl_old.shape)
    x_let, y_let = ic.process_data(test_dir, 1000, 'letter')
    print(x_let.shape, y_let.shape)
    print(letter_model.metrics_names)
    print(letter_model.evaluate(x=x_let, y=y_let))
    # path = '../targets/output/old/%03d.jpg'%i
    path = '../targets/output/test2/%05d.jpg'%i
    print(vc.predict(letter_model, vc.letter_img(path)))
    print('predicted: ', vc.predict_letter(letter_model, vc.letter_img(path)))
    # print('true: ', chr(yl_old[i].argmax() + 65))
    print('true: ', vc.LETTERS[y_let[i].argmax()])

def eval_models(i=1):
    eval_shape_model(i)
    eval_letter_model(i)

if __name__=='__main__':
    train_letter_model(10)
    #eval_models(1)

