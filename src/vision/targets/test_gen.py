import random
import os
import sys
import numpy as np
import gen
import targets
sys.path.insert(0, '../classifier')
import segmentation.letter_seg as ls 

ROTATIONS = [
    0, 45, 90, 135, 180, 225, 270, 315
]

"""
Generates a set of training data, along with labels. The formatting
is weird since I'm just testing the target generator/training out
and I can't be bothered to worry about that.
"""
def generate_set(n, name):
    labels = np.zeros((n, 3), dtype=np.int32)
    output_dest = os.path.join('output', name)
    #data_dest = os.path.join('output', 'data_' + name)
    os.makedirs(output_dest, exist_ok=True)
    #os.makedirs(data_dest, exist_ok=True)
    for i in range(n):
        # randomly select and record some parameters
        shape = random.randint(0, len(targets.TARGET_TYPES) - 1)
        letter = random.randint(0, len(gen.LETTERS) - 1)
        angle = random.randint(0, len(ROTATIONS) - 1)
        labels[i] = [shape, letter, angle]
        # generate a single image
        generator = gen.SingleTarget(
            target_name=targets.TARGET_TYPES[shape], 
            font='fonts/OpenSans-Bold.ttf')
        # these should match the defaults in gen.py, except when modified
        gen.gen_images(
            t_gen=            generator,
            n=                1,
            img_name=         str(i),
            letter=           gen.LETTERS[letter],
            shape=            targets.TARGET_TYPES[shape],
            white_balance=    None,
            t_size=           100,
            i_size=           (100, 100),
            r_angle=          ROTATIONS[angle],
            bg_dir=           'backgrounds',
            dest_dir=         output_dest,
            transforms=       [],
            draw_box=         None,
            rescale_ratio=    1,
            origin_pos=       (0, 0),
            shape_color_name= 'random',
            letter_color_name='random',
            target_pos_conf=  None)
        # dump converted image to file
        #img_path = os.path.join(output_dest, str(i) + '.jpg')
        #data_path = os.path.join(data_dest, 'img_' + str(i) + '.csv')
        #np.savetxt(data_path, ls.letter_seg(img_path), delimiter=',')
    # dump labels to file
    np.savetxt(os.path.join(output_dest, 'labels.csv'), labels, delimiter=',')

if __name__=='__main__':
    generate_set(100, 'train')
    generate_set(10, 'test')

