import targets
from PIL import Image, ImageDraw
from argparse import ArgumentParser
import random
import csv
import os
import math
import urllib.request
import subprocess

# yapf: disable
COLORS = {
    'white':  (0, (255, 255, 255)),
    'black':  (1, (79,  86,  100)),
    'gray':   (2, (128, 128, 128)),
    'red':    (3, (237, 73,  98)),
    'blue':   (4, (55,  129, 205)),
    'green':  (5, (103, 162, 114)),
    'yellow': (6, (245, 223, 20)),
    'purple': (7, (108, 73,  155)),
    'brown':  (8, (219, 187, 137)),
    'orange': (9, (250, 149, 41))
}
# yapf: enable

KELVIN_TEMP = {
    4500: (255, 219, 186),
    5000: (255, 228, 206),
    5500: (255, 236, 224),
    6000: (255, 243, 239),
    6500: (255, 249, 253),
    7000: (245, 243, 255),
    7500: (235, 238, 255),
    8000: (227, 233, 255),
    8500: (220, 229, 255),
    9000: (214, 225, 255),
    9500: (208, 222, 255),
    10000: (204, 219, 255)
}

LETTERS = [
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
    'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '0', '1', '2', '3',
    '4', '5', '6', '7', '8', '9'
]
# LETTERS = [ # old set missing numbers
#     'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
#     'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 
# ]

TRANSFORMS = ('rotate', 'perspective', 'affine')


class TargetSelector:
    def __init__(self, target_names, font):
        self.targets = []
        for target_name in target_names:
            self.targets += [getattr(targets, target_name)(font)]

    def __iter__(self):
        return self

    def __next__(self):
        return random.choice(self.targets)


class SingleTarget:
    def __init__(self, target_name, font):
        self.target = getattr(targets, target_name)(font)

    def __iter__(self):
        return self

    def __next__(self):
        return self.target


def gen_images(t_gen, n, letter, t_size, i_size, r_angle, bg_dir, dest_dir,
               transforms, draw_box, rescale_ratio, shape_color_name,
               letter_color_name, target_pos_conf, white_balance, origin_pos,
               start_num, file_digits):
    background_files = os.scandir(bg_dir)
    field_width = file_digits
    if file_digits is None:
        field_width = math.ceil(math.log10(n + start_num))
    labels = []
    for i in range(n):
        if i % 100 == 0:
            print('Generating image #{}'.format(i))
        # Get a background image
        background_file = None
        try:
            background_file = next(background_files).path
        except StopIteration:
            # if there are no more files, go back to the beginning
            background_files = os.scandir(bg_dir)
            background_file = next(background_files).path
        background = Image.open(background_file)

        # Rescale background
        if rescale_ratio != 1:
            background = background.resize(
                (int(background.width * rescale_ratio),
                 int(background.height * rescale_ratio)),
                resample=Image.BICUBIC)

        # Choose some colors
        shape_color_choice = random.choice(list(COLORS.values(
        ))) if shape_color_name == 'random' else COLORS[shape_color_name]

        letter_color_choice = random.choice(list(COLORS.values(
        ))) if letter_color_name == 'random' else COLORS[letter_color_name]

        while (shape_color_choice == letter_color_choice
               and shape_color_name == 'random'
               and letter_color_name == 'random'):
            shape_color_choice = random.choice(list(COLORS.values()))

        pure_shape_color = shape_color_choice[1]
        pure_letter_color = letter_color_choice[1]

        # randomize rgb values for the colors chosen
        shape_color_lst = []
        for value in pure_shape_color:
            if value == 255:
                shape_color_lst.append(value + int(random.uniform(-15, 0)))
            elif value == 0:
                shape_color_lst.append(value + int(random.uniform(0, 15)))
            else:
                shape_color_lst.append(value + int(random.uniform(-15, 15)))

        letter_color_lst = []
        for value in pure_letter_color:
            if value == 255:
                letter_color_lst.append(value + int(random.uniform(-15, 0)))
            elif value == 0:
                letter_color_lst.append(value + int(random.uniform(0, 15)))
            else:
                letter_color_lst.append(value + int(random.uniform(-15, 15)))

        shape_color = tuple(shape_color_lst)
        letter_color = tuple(letter_color_lst)

        # Draw a target with a generator decided by the iterator
        generator = next(t_gen)
        if (letter == 'Random'):
            letter_choice = random.choice(LETTERS)
        else:
            letter_choice = letter
        target = generator.draw_target(
            size=t_size,
            shape_color=shape_color,
            letter=letter_choice,
            letter_color=letter_color)
        if r_angle > 0:
            target = target.rotate(
                r_angle, resample=Image.BICUBIC, expand=1).resize(
                    (t_size, t_size), resample=Image.BOX)
        if 'rotate' in transforms:
            r_angle = random.randint(0, 359)
            target = target.rotate(
                r_angle, resample=Image.BICUBIC,
                expand=1).resize((t_size, t_size), resample=Image.BOX)

        if target_pos_conf is None:
            target_pos = (random.randint(0, i_size[0] - t_size),
                          random.randint(0, i_size[1] - t_size))
        else:
            target_pos = target_pos_conf

        # create annotation
        im_filename = ('{:0=' + str(field_width) + 'd}').format(i + start_num)
        target_bounds = [(target_pos[0], target_pos[1]),
                         (target_pos[0] + t_size, target_pos[1] + t_size)]
        labels.append([
            i, # 0, name
            i_size[0], # 1, width
            i_size[1], # 2, height
            3, # 3, depth
            r_angle, # 4, rotation
            targets.TARGET_TYPES.index(type(generator).__name__), # 5, shape
            LETTERS.index(letter_choice), # 6, letter
            shape_color_choice[0], # 7, shape color
            letter_color_choice[0], # 8, letter color
            target_bounds[0][0], # 9, xmin
            target_bounds[0][1], # 10, ymin
            target_bounds[1][0], # 11, xmax
            target_bounds[1][1] # 12, ymax
        ])

        # Draw a bounding box
        if draw_box:
            draw = ImageDraw.Draw(background)
            draw.rectangle(target_bounds, outline=(255, 255, 255, 255))

        # Paste the target and save
        ori_x, ori_y = origin_pos
        if ori_x + i_size[0] > background.width:
            ori_x = background.width - i_size[0]
        if ori_y + i_size[1] > background.height:
            ori_y = background.height - i_size[1]
        result = background.crop(
            box=(ori_x, ori_y, ori_x + i_size[0], ori_y + i_size[1]))

        result.paste(target, box=target_pos, mask=target)
        if white_balance:
            r, g, b = random.choice(tuple(KELVIN_TEMP.values()))
            convert_temp = (r / 255.0, 0.0, 0.0, 0.0, 0.0, g / 255.0, 0.0, 0.0,
                            0.0, 0.0, b / 255.0, 0.0)
            result = result.convert('RGB', convert_temp)
        result.save(
            os.path.join(dest_dir,
                         im_filename + '.' + background_file.split('.')[1]))

    with open(os.path.join(dest_dir, 'labels.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'name', 'width', 'height', 'depth', 'rotation', 'shape', 'letter',
            'shape_color', 'letter_color', 'xmin', 'ymin', 'xmax', 'ymax'
        ])
        writer.writerows(labels)


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument(
        '-n',
        '--number',
        type=int,
        default=1,
        dest='n_targets',
        help='number of targets to generate')
    parser.add_argument(
        '-l',
        '--letter',
        default='Random',
        dest='letter',
        choices=LETTERS + ['Random'],
        help='type of letter to generate')
    parser.add_argument(
        '-s',
        '--shape',
        default='Random',
        dest='target_shape',
        choices=targets.TARGET_TYPES + ['Random'],
        help='type of target to generate')
    parser.add_argument(
        '--shape-color',
        default='random',
        dest='shape_color',
        choices=tuple(COLORS.keys()),
        help='name of the color of the shape')
    parser.add_argument(
        '--letter-color',
        default='random',
        dest='letter_color',
        choices=tuple(COLORS.keys()),
        help='name of the color of the letter')
    parser.add_argument(
        '-w',
        '--white-balance',
        dest='white_balance',
        action='store_true',
        help='simulate incorrect white balance')
    parser.add_argument(
        '--target-size',
        type=int,
        default=100,
        dest='target_size',
        help='width/height of target(s) in pixels')
    parser.add_argument(
        '--target-pos',
        type=int,
        nargs=2,
        default=None,
        dest='target_pos',
        help='position of the target on the image, by default it is random')
    parser.add_argument(
        '--image-size',
        type=int,
        nargs=2,
        default=None,
        dest='image_size',
        help='width and height of the image as a tuple (width, height). \
        By default it will crop the image to the target')
    parser.add_argument(
        '--rotation-angle',
        type=int,
        default=0,
        dest='rotation_angle',
        help='angle to rotate the image by, in degrees.')
    parser.add_argument(
        '-d',
        '--dest',
        default=None,
        dest='dest',
        help='destination folder of the target images; by default it is the \
        name of the target shape')
    parser.add_argument(
        '-f',
        '--font',
        default=None,
        dest='font',
        help='path to the font to use for the targets; leave unspecified to \
        automatically download a font')
    parser.add_argument(
        '-b',
        '--backgrounds',
        default='backgrounds',
        dest='backgrounds',
        help='path to folder containing background images')
    parser.add_argument(
        '--transforms',
        nargs='*',
        default=[],
        choices=TRANSFORMS,
        help='transformations to apply to the targets and images')
    parser.add_argument(
        '--draw-box',
        action='store_true',
        dest='draw_box',
        help='draw a bounding box around the target')
    parser.add_argument(
        '--rescale-bg',
        type=float,
        default=1,
        dest='rescale_ratio',
        help='rescale the source image before using it as a background')
    parser.add_argument(
        '--origin-pos',
        type=int,
        nargs=2,
        default=(0, 0),
        dest='origin_pos',
        help='x, y coordinate of the alternative origin to crop the image as a \
        tuple. By default it is (0,0) and max they can get is the width and \
        height of the background minus image_size')
    parser.add_argument(
        '--start-num',
        type=int,
        default=0,
        help='numbering offset for the image file names')
    parser.add_argument(
        '--file-digits',
        type=int,
        default=None,
        help='number of digits for the image file names')

    args = parser.parse_args()

    if args.dest is not None:
        args.dest = os.path.realpath(args.dest)
    args.backgrounds = os.path.realpath(args.backgrounds)

    os.chdir(os.path.dirname(os.path.realpath(__file__)))

    if args.n_targets < 1:
        parser.error('number of targets must be positive')
    if args.image_size is None:
        args.image_size = (args.target_size, args.target_size)
    if args.font is None:
        if not os.path.isfile('fonts/OpenSans-Bold.ttf'):
            os.makedirs('fonts', exist_ok=True)
            print('Downloading necessary fonts...')
            urllib.request.urlretrieve(
                'https://www.fontsquirrel.com/fonts/download/open-sans',
                'fonts/open-sans.zip')
            print('Extracting fonts...')
            subprocess.run(['unzip', 'fonts/open-sans.zip', '-d', 'fonts'])
            print('Fonts extracted!')
        args.font = 'fonts/OpenSans-Bold.ttf'
    if args.dest is None:
        default_dest = os.path.join('output', args.target_shape)
        os.makedirs(default_dest, exist_ok=True)
        args.dest = default_dest
    else:
        os.makedirs(args.dest, exist_ok=True)

    if args.target_shape == 'Random':
        generator = TargetSelector(
            target_names=targets.TARGET_TYPES, font=args.font)
    else:
        generator = SingleTarget(target_name=args.target_shape, font=args.font)

    gen_images(
        t_gen=generator,
        n=args.n_targets,
        letter=args.letter,
        white_balance=args.white_balance,
        t_size=args.target_size,
        i_size=args.image_size,
        r_angle=args.rotation_angle,
        bg_dir=args.backgrounds,
        dest_dir=args.dest,
        transforms=args.transforms,
        draw_box=args.draw_box,
        rescale_ratio=args.rescale_ratio,
        origin_pos=args.origin_pos,
        shape_color_name=args.shape_color,
        letter_color_name=args.letter_color,
        target_pos_conf=args.target_pos,
        start_num=args.start_num,
        file_digits=args.file_digits)
