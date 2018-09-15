import targets
from PIL import Image
from argparse import ArgumentParser
import random
import os
import math

COLORS = {
    'white': (255, 255, 255),
    'black': (0, 0, 0),
    'gray': (128, 128, 128),
    'red': (255, 0, 0),
    'blue': (0, 0, 255),
    'green': (0, 255, 0),
    'yellow': (255, 255, 0),
    'purple': (128, 0, 128),
    'brown': (165, 42, 42),
    'orange': (255, 165, 0)
}

LETTERS = [
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
    'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'
]


def gen_images(t_gen, n, shape, t_size, i_size, bg_dir, dest_dir):
    background_files = os.scandir(bg_dir)
    field_width = math.trunc(math.log10(n))
    for i in range(n):

        # Get a background image
        background_file = None
        try:
            background_file = next(background_files).path
        except StopIteration:
            # if there are no more files, go back to the beginning
            background_files = os.scandir(bg_dir)
            background_file = next(background_files).path
        background = Image.open(background_file)

        # Choose some colors
        shape_color = random.choice(list(COLORS.values()))
        letter_color = random.choice(list(COLORS.values()))
        while letter_color == shape_color:
            letter_color = random.choice(list(COLORS.values()))

        # Draw a target with a generator decided by the iterator
        generator = next(t_gen)
        target = generator.draw_target(
            size=t_size,
            shape_color=shape_color,
            letter=random.choice(LETTERS),
            letter_color=letter_color)
        target_pos = (random.randint(0, i_size[0] - t_size),
                      random.randint(0, i_size[1] - t_size))

        # Paste the target and save
        result = background.crop(box=(0, 0, i_size[0], i_size[1]))
        result.paste(target, box=target_pos, mask=target)
        result.save(
            os.path.join(dest_dir,
                         ('{:0=' + str(field_width) + 'd}.jpg').format(i)))


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
        '-s',
        '--shape',
        default='Random',
        dest='target_shape',
        choices=targets.TARGET_TYPES + ['Random'],
        help='type of target to generate')
    parser.add_argument(
        '--target-size',
        type=int,
        default=100,
        dest='target_size',
        help='width/height of target(s) in pixels')
    parser.add_argument(
        '--image-size',
        type=int,
        nargs=2,
        default=None,
        dest='image_size',
        help=
        'width and height of the image as a tuple (width, height). By default it will crop the image to the target'
    )
    parser.add_argument(
        '-d',
        '--dest',
        default=None,
        dest='dest',
        help=
        'destination folder of the target images; by default it is the name of the target shape'
    )
    parser.add_argument(
        '-f',
        '--font',
        default=None,
        dest='font',
        help=
        'path to the font to use for the targets; leave unspecified to automatically download a font'
    )
    parser.add_argument(
        '-b',
        '--backgrounds',
        default='backgrounds',
        dest='backgrounds',
        help='path to folder containing background images')

    args = parser.parse_args()

    if args.n_targets < 1:
        parser.error('number of targets must be positive')
    if args.image_size is None:
        args.image_size = (args.target_size, args.target_size)
    if args.font is None:
        # TODO add font autodownload
        args.font = 'arialbd.ttf'
    if args.dest is None:
        default_dest = os.path.join('output', args.target_shape)
        os.makedirs(default_dest, exist_ok=True)
        #if not os.path.isdir(args.target_shape):
        #    os.mkdir(args.target_shape)
        args.dest = default_dest

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

    if args.target_shape == 'Random':
        generator = TargetSelector(
            target_names=targets.TARGET_TYPES, font=args.font)
    else:
        generator = SingleTarget(target_name=args.target_shape, font=args.font)

    gen_images(
        t_gen=generator,
        n=args.n_targets,
        shape=args.target_shape,
        t_size=args.target_size,
        i_size=args.image_size,
        bg_dir=args.backgrounds,
        dest_dir=args.dest)
