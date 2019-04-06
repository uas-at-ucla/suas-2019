from PIL import Image
from argparse import ArgumentParser
from sys import exit
from math import ceil
import os


def tile(source_path, dest_path, tile, use_res):
    source = Image.open(source_path)

    if use_res:
        dest = Image.new(mode='RGB', size=(tile[0], tile[1]))
        # the title range must be the ceiling of the output res / source res
        tile_range = (range(ceil(tile[0] / source.width)),
                      range(ceil(tile[1] / source.height)))
    else:
        if tile[0] > 50 or tile[1] > 50:
            print("Are you sure you want to tile {}x{} images?".format(*tile))
            exit(1)
        dest = Image.new(
            mode='RGB', size=(source.width * tile[0], source.height * tile[1]))
        tile_range = (range(tile[0]), range(tile[1]))

    for x in tile_range[0]:
        for y in tile_range[1]:
            dest.paste(source, box=(x * source.width, y * source.height))
    dest.save(dest_path)


if __name__ == '__main__':
    parser = ArgumentParser()

    parser.add_argument('source_file', help='the source tile or folder')
    parser.add_argument('dest_file', help='the destination file or folder')
    parser.add_argument(
        '--tile',
        nargs=2,
        type=int,
        default=(3, 4),
        help='(#x-tiles, #y-tiles) the number of tiles in each direction')
    parser.add_argument(
        '--use-res',
        action='store_true',
        help='tile until resolution is reached. '
        'Resolution is defined with the --tile argument')

    args = parser.parse_args()

    if os.path.isdir(args.source_file):
        if not os.path.isdir(args.dest_file):
            print('Dest must be a directory if the source is a directory.')
            exit(1)
        sources = os.scandir(args.source_file)

        for source in sources:
            dest_path = os.path.join(args.dest_file, source.name)
            tile(source.path, dest_path, args.tile, args.use_res)
    else:
        tile(args.source_file, args.dest_file, args.tile, args.use_res)
