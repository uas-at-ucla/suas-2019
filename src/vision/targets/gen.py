import targets
from PIL import Image, ImageDraw
from argparse import ArgumentParser
import random
import os
import math
import urllib.request
import subprocess
import xml.etree.ElementTree as ET

# run this script only in the targets directory
os.chdir(os.path.dirname(os.path.realpath(__file__)))

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

TRANSFORMS = ('rotate', 'perspective', 'affine')


def gen_images(t_gen, n, shape, t_size, i_size, bg_dir, dest_dir, transforms,
               draw_box):
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
        if 'rotate' in transforms:
            target = target.rotate(
                random.randint(0, 359), resample=Image.BICUBIC,
                expand=1).resize((t_size, t_size), resample=Image.BOX)

        target_pos = (random.randint(0, i_size[0] - t_size),
                      random.randint(0, i_size[1] - t_size))

        # create annotation
        im_filename = ('{:0=' + str(field_width) + 'd}').format(i)
        annotation = ET.Element('annotation')
        annotation_xml = ET.ElementTree(element=annotation)
        ET.SubElement(annotation, 'filename').text = im_filename + '.jpg'
        ET.SubElement(ET.SubElement(annotation, 'source'),
                      'database').text = 'Unknown'
        size_e = ET.SubElement(annotation, 'size')
        ET.SubElement(size_e, 'width').text = str(i_size[0])
        ET.SubElement(size_e, 'height').text = str(i_size[1])
        ET.SubElement(size_e, 'depth').text = '3'
        ET.SubElement(annotation, 'segmented').text = '0'
        obj_e = ET.SubElement(annotation, 'object')
        ET.SubElement(obj_e, 'name').text = type(generator).__name__
        ET.SubElement(obj_e, 'pose').text = 'Unspecified'
        ET.SubElement(obj_e, 'truncated').text = '0'
        ET.SubElement(obj_e, 'difficult').text = '0'

        target_bounds = [(target_pos[0], target_pos[1]),
                         (target_pos[0] + t_size, target_pos[1] + t_size)]
        bnds_e = ET.SubElement(obj_e, 'bndbox')
        ET.SubElement(bnds_e, 'xmin').text = str(target_bounds[0][0])
        ET.SubElement(bnds_e, 'ymin').text = str(target_bounds[0][1])
        ET.SubElement(bnds_e, 'xmax').text = str(target_bounds[1][0])
        ET.SubElement(bnds_e, 'ymax').text = str(target_bounds[1][1])
        annotation_xml.write(os.path.join(dest_dir, im_filename + '.xml'))

        # Draw a bounding box
        if draw_box:
            draw = ImageDraw.Draw(background)
            draw.rectangle(target_bounds, outline=(255, 255, 255, 255))

        # Paste the target and save
        result = background.crop(box=(0, 0, i_size[0], i_size[1]))
        result.paste(target, box=target_pos, mask=target)
        result.save(os.path.join(dest_dir, im_filename + '.jpg'))


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
        help='width and height of the image as a tuple (width, height). \
        By default it will crop the image to the target')
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

    args = parser.parse_args()

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
        dest_dir=args.dest,
        transforms=args.transforms,
        draw_box=args.draw_box)
