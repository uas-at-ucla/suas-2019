#!/usr/bin/env python

from __future__ import division

import sys
sys.dont_write_bytecode = True

import TargetGenerator
import TargetGenerator.global_settings as gs
import shutil
import cv2
import glob
import os
import argparse
import random

def convertMeterToPixel(meter1,meter2,zoomLevel):
    if zoomLevel == 18:
        return meter1 / 0.3, meter2 / 0.3
    elif zoomLevel == 19:
        return meter1 / 0.15, meter2 / 0.15
    elif zoomLevel == 20:
        return meter1/ 0.075, meter2 / 0.075
    else:
        return 0,0

def main(visualize):
    # Create paths
    imgs_paths = sorted(
        glob.glob(os.path.join(gs.DATA_PATH, 'gmap_tiles', '*.jpg')))
    img_names = [
        os.path.splitext(os.path.split(path)[1])[0] for path in imgs_paths
    ]
    if len(imgs_paths) == 0:
        raise ValueError("ERROR: No background tiles loaded (gmap_tiles)")

    data_paths = [
        os.path.join(gs.DATA_PATH, 'flight_data', 'data.json')
        for name in img_names
    ]

    dst_folder = os.path.join(gs.DATA_PATH, 'train_images')

    # Prepare empty destination folder (where training data will be stored).
    if os.path.exists(dst_folder):
        shutil.rmtree(dst_folder)

    os.makedirs(dst_folder)

    # Load image and image data
    img_index = 0
    for shape_img_path, shape_data_path, empty_img_path, empty_data_path in zip(
            imgs_paths, data_paths, imgs_paths, data_paths):

        shape_img = TargetGenerator.Image(
            shape_img_path, shape_data_path, K=gs.resized_K)

        shape_patches = shape_img.createFullPatch()
#       shape_patches = shape_img.createPatches(
#           patch_size=gs.PATCH_SIZE, patch_shift=1)

        for patch in shape_patches:
            if img_index > 5000:
                break

            # Paste a random target on the patch
            current_altitude = random.randint(25, 30)
            target, target_label, _, shape = TargetGenerator.randomTarget(
                altitude=current_altitude,
                longitude=shape_img.longitude,
                latitude=shape_img.latitude)

            coords = shape_img.pastePatch(patch=patch, target=target)
            coords = TargetGenerator.squareCoords(coords, noise=False)
            original_patch = patch.copy()

            patch = cv2.resize(patch, dsize=gs.CLASSIFIER_PATCH_SIZE)

            print("Created img " + shape + " with bounding box (" \
                    + str(coords[0]) + ", " \
                    + str(coords[1]) + "), (" \
                    + str(coords[2]) + ", " \
                    + str(coords[3]) + ")")

            if visualize:
                cv2.rectangle(original_patch, (coords[0], coords[1]),
                              (coords[2], coords[3]), (0, 255, 0), 1)
                cv2.namedWindow('patch', flags=cv2.WINDOW_NORMAL)
                cv2.imshow('patch', original_patch)
                cv2.waitKey(0)

            filename = '{:07}'.format(img_index)

            label = \
"<annotation>\n" \
"\t<filename>" + filename + ".jpg</filename>\n" \
"\t<source>\n" \
"\t\t<database>Unknown</database>\n" \
"\t</source>\n" \
"\t<size>\n" \
"\t\t<width>" + str(gs.CLASSIFIER_PATCH_SIZE[0]) + "</width>\n" \
"\t\t<height>" + str(gs.CLASSIFIER_PATCH_SIZE[1]) + "</height>\n" \
"\t\t<depth>3</depth>\n" \
"\t</size>\n" \
"\t<segmented>0</segmented>\n" \
"\t<object>\n" \
"\t\t<name>" + shape + "</name>\n" \
"\t\t<pose>Unspecified</pose>\n" \
"\t\t<truncated>0</truncated>\n" \
"\t\t<difficult>0</difficult>\n" \
"\t\t<bndbox>\n" \
"\t\t\t<xmin>" + str(coords[0] * 2) + "</xmin>\n" \
"\t\t\t<ymin>" + str(coords[1] * 2) + "</ymin>\n" \
"\t\t\t<xmax>" + str(coords[2] * 2) + "</xmax>\n" \
"\t\t\t<ymax>" + str(coords[3] * 2) + "</ymax>\n" \
"\t\t</bndbox>\n" \
"\t</object>\n" \
"</annotation>"

            img_index += 1

            cv2.imwrite(os.path.join(dst_folder, filename + '.jpg'), patch)
            with open(os.path.join(dst_folder, filename + '.xml'), 'w') as fp:
                fp.write(label)

    if visualize:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    cmdline = argparse.ArgumentParser(
        usage="usage: ./{}".format(os.path.basename(__file__)),
        description="Create target patches")

    cmdline.add_argument(
        "--visualize",
        action="store_true",
        help="Visualize outputs.",
        dest="visualize",
        default=False)

    args = cmdline.parse_args()

    main(visualize=args.visualize)
