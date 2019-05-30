import os
from target_analysis import *

def test_all_in_dir(in_dir, out_dir, display=True):
    for image in os.listdir(in_dir):
        name = os.path.splitext(image)[0]
        extension = os.path.splitext(image)[1]
        if (extension == '.jpg'):
            print(filename)
            analyze_image(in_dir, name, out_dir=out_dir, display=display, debug=False)


if __name__=='__main__':
    os.makedirs('output/data', exist_ok=True)
    test_all_in_dir('data', 'output/data', False)

