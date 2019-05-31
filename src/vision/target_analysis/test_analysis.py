import os
from target_analysis import *

def test_all_in_dir(in_dir, out_dir, display=True):
    for image in os.listdir(in_dir):
        name = os.path.splitext(image)[0]
        extension = os.path.splitext(image)[1]
        if (extension == '.jpg'):
            analyze_one_image(in_dir, int(name), out_dir=out_dir, display=display, debug=True)


if __name__=='__main__':
    os.makedirs('output/data', exist_ok=True)
    os.makedirs('output/data/histograms', exist_ok=True)
    os.makedirs('output/data/countours', exist_ok=True)
    os.makedirs('output/data/thresholds', exist_ok=True)
    test_all_in_dir('data', 'output/data', False)

