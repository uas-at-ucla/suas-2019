#!/usr/bin/env python

# Modified target generator for only cropped targets (for shape classification training)

from __future__ import division
import TargetGenerator
import TargetGenerator.global_settings as gs
import shutil
import cv2
import glob
import os
import argparse
import random

TARGET_INDICIES = {
    "random": None,
    "circle": 0,
    "semicircle": 1,
    "quarter_circle": 2,
    "rectangle": 3,
    "trapezoid": 4,
    "triangle": 5,
    "cross": 6,
    "pentagon": 7,
    "hexagon": 8,
    "heptagon": 9,
    "octagon": 10,
    "star": 11,
    "blank": 12
}


def main(visualize, n_images, selected_target, overwrite, dest_folder):
    #
    # Create paths
    #

    # Field image path
    imgs_paths = sorted(
        glob.glob(os.path.join(gs.DATA_PATH, 'gmap_tiles', '*.jpg')))
    img_names = [
        os.path.splitext(os.path.split(path)[1])[0] for path in imgs_paths
    ]
    if len(imgs_paths) == 0:
        raise ValueError("ERROR: No background tiles loaded (gmap_tiles)")

    # Black tile path
    black_tile_paths = sorted(
        glob.glob(os.path.join(gs.DATA_PATH, 'black_tile', '*.png')))
        # glob.glob(os.path.join(gs.DATA_PATH, 'gmap_tiles', '*.jpg')))
    black_tile_names = [
        os.path.splitext(os.path.split(path)[1])[0] for path in black_tile_paths 
    ]
    if len(black_tile_paths) == 0:
        raise ValueError("ERROR: No background tiles loaded (black_tile)")

    data_paths = [
        os.path.join(gs.DATA_PATH, 'flight_data', 'data.json')
        for name in img_names
    ]

    if dest_folder == None:
        if selected_target == None:
            dst_folder = os.path.join(gs.DATA_PATH, 'train_images', 'random')
        else:
            dst_folder = os.path.join(gs.DATA_PATH, 'train_images', selected_target)
    else:
        dst_folder = os.path.join(gs.DATA_PATH, dest_folder)

    # Prepare empty destination folder (where training data will be stored).
    if os.path.exists(dst_folder):
        if len(os.listdir(dst_folder)) != 0:
            if overwrite:
                shutil.rmtree(dst_folder)
                os.makedirs(dst_folder)
                os.makedirs(os.path.join(dst_folder, 'ground_truth'))
            else:
                raise IOError("Destination folder '" + dst_folder + "' not empty, use '-f' to force overwrite.")
    else:
        os.makedirs(dst_folder)

    #
    # Load image and image data
    #
    with_target_flag = 0
    img_index = 0
    for shape_img_path, shape_data_path, empty_img_path, empty_data_path in zip(
        imgs_paths,
        data_paths,
        imgs_paths,
        data_paths):

        shape_img = TargetGenerator.Image(shape_img_path,
            shape_data_path,
            K=gs.resized_K)

        shape_patches = shape_img.createFullPatch()

        for patch in shape_patches:

            if img_index > n_images - 1:
                break

            letter = random.choice(gs.LETTERS)
            orientation = random.random() * 360
            target, target_label, _, shape= TargetGenerator.genTarget(
                altitude=0,
                longitude=shape_img.longitude,
                latitude=shape_img.latitude,
                target_label=TARGET_INDICIES[selected_target],
                # color_letter=(255, 255, 255),
                letter_label=letter,
                orien=orientation
            )

            # make a blank target
            blank_target, _, _, _ = TargetGenerator.genTarget(
                altitude=0,
                longitude=shape_img.longitude,
                latitude=shape_img.latitude,
                target_label=TARGET_INDICIES["blank"],
                color_letter=(255, 255, 255),
                letter_label=letter,
                orien=orientation
            )

            # paste the targets
            coords = shape_img.pastePatch(patch=patch, target=target)
            coords = TargetGenerator.squareCoords(coords, noise=False)
            patch = patch[coords[1]:coords[3], coords[0]:coords[2], ...]

            # patch = cv2.resize(patch, dsize=gs.CLASSIFIER_PATCH_SIZE)

            # paste the blank target
            black_img = TargetGenerator.Image(black_tile_paths[0], shape_data_path, K=gs.resized_K)
            black_patches = black_img.createFullPatch()
            black_patch = black_patches.next()

            black_coords = black_img.pastePatch(patch=black_patch, target=blank_target)
            black_coords = TargetGenerator.squareCoords(black_coords, noise=False)

            black_patch = black_patch[black_coords[1]:black_coords[3], black_coords[0]:black_coords[2], ...]
            _, black_patch = cv2.threshold(black_patch, 127, 255, cv2.THRESH_BINARY)
            # black_patch = cv2.resize(black_patch, dsize=gs.CLASSIFIER_PATCH_SIZE)

            if visualize:
                cv2.namedWindow('original patch', flags=cv2.WINDOW_NORMAL)
                cv2.imshow('original patch', original_patch)

                cv2.namedWindow('patch', flags=cv2.WINDOW_NORMAL)
                cv2.imshow('patch', patch)
                cv2.waitKey(0)

            label = target_label

            filename = '{:07}'.format(img_index)
            img_index += 1

            cv2.imwrite(os.path.join(dst_folder, filename+'.jpg'), patch)
            cv2.imwrite(os.path.join(dst_folder, 'ground_truth',  filename+'.png'), black_patch)
            #with open(os.path.join(dst_folder, filename+'.txt'), 'w') as fp:
            #    fp.write('{}.jpg\t{}'.format(filename, label))

    if visualize:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    cmdline = argparse.ArgumentParser(usage="usage: ./{}"
                                            .format(os.path.basename(__file__)),
                                      description="Create target patches")

    cmdline.add_argument("--visualize",
                         action="store_true",
                         help="Visualize outputs.",
                         dest="visualize",
                         default=False)

    cmdline.add_argument("-n", type=int, default=10, dest="n_images", help="Number of images to generate.")
    cmdline.add_argument("-t", type=str, default="random", dest="selected_target", help="Type of target to generate.")
    cmdline.add_argument("-f", action="store_true", dest="overwrite", default=False, help="Overwrite existing targets in destination.")
    cmdline.add_argument("--dest", type=str, default=None, dest="dest_folder", help="Specify destination folder for output.")

    args = cmdline.parse_args()

    main(visualize=args.visualize, n_images=args.n_images, selected_target=args.selected_target, overwrite=args.overwrite, dest_folder=args.dest_folder)
