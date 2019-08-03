import os
from identify_colors import mask_image

def test_all_in_dir(dir, out_dir, display=True):
    for image in os.listdir(dir):
        name = os.path.splitext(image)[0]
        extension = os.path.splitext(image)[1]
        filename = dir + name + extension
        
        if (extension == '.jpg'):
            print(filename)
            mask_image(filename, out_dir=out_dir, display=display)

if __name__ == '__main__':
    # test_all_in_dir('images/test2/', 'output/test2/', False)
    test_all_in_dir('images/train_many/', 'output/train_many/', False)

