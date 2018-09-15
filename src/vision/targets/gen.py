import targets
from PIL import Image
from argparse import ArgumentParser
import random


def gen_images(t_gen, n, shape, t_size, i_size, backs, dest):
    pass


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
        type=tuple,
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

    class TargetSelector:
        def __init__(self, target_names, font):
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
        backs=args.backgrounds,
        dest=args.dest)
