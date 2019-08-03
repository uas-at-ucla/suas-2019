import os
from target_analysis import *

def test_all_in_dir(in_dir, out_dir, display=True):
    for i in range(100):
        analyze_one_image(in_dir, i, out_dir=out_dir, display=display, debug=True)


if __name__=='__main__':
    in_dir = 'data/large'
    out_dir = 'output/large'
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'histograms'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'countours'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'thresholds'), exist_ok=True)
    test_all_in_dir(in_dir, out_dir, False)

