# ===================================================
# This script parses the generated annotation files
# and compiles them into a csv file
# ===================================================
import xml.etree.ElementTree as ET
import csv
import os
from sys import exit
from argparse import ArgumentParser


def parse_and_write(folder, csvfile, append):
    csvwriter = csv.writer(csvfile)
    if not append:
        csvwriter.writerow([
            'filename', 'width', 'height', 'depth', 'name', 'xmin', 'ymin',
            'xmax', 'ymax'
        ])
    for file in os.scandir(folder):
        if file.name.split('.')[1] != 'xml':
            continue

        tree = ET.parse(file.path)
        im_name = tree.find('filename').text
        width = tree.find('size').find('width').text
        height = tree.find('size').find('height').text
        depth = tree.find('size').find('depth').text
        obj_name = tree.find('object').find('name').text
        xmin = tree.find('object').find('bndbox').find('xmin').text
        ymin = tree.find('object').find('bndbox').find('ymin').text
        xmax = tree.find('object').find('bndbox').find('xmax').text
        ymax = tree.find('object').find('bndbox').find('ymax').text

        csvwriter.writerow(
            [im_name, width, height, depth, obj_name, xmin, ymin, xmax, ymax])


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument(
        '--no-append',
        action='store_false',
        help='overwrite the target csv file;'
        'by default it will append to the csv file if it exists')
    parser.add_argument('image_dir', help='path to the images + annotations')
    parser.add_argument('csv_file', help='path to the csv file to write to')
    args = parser.parse_args()

    folder = args.image_dir
    csvfilepath = args.csv_file
    append = args.no_append

    if not append:
        try:
            with open(csvfilepath, 'x') as csvfile:
                parse_and_write(folder, csvfile, append)
        except FileExistsError:
            print('"{}" already exists, but --no-append option was selected.'.
                  format(csvfilepath))
            exit(1)
    else:
        with open(csvfilepath, 'a') as csvfile:
            parse_and_write(folder, csvfile, append)
