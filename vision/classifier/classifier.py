import photographer
import segmenter

from collections import deque
import os
import sys
import thread
import time

def form_directory_structure():
    # Create a neat directory structure to store all of the image that our
    # image identification system will generate.
    root = "/home/pi/vision/"
    if len(sys.argv) > 1:
        root = sys.argv[1]

    parent_directory = root + "captures"
    if os.path.isdir(parent_directory) is False:
        os.makedirs(parent_directory)
        print "Created captures directory at " + parent_directory

    raw_directory = parent_directory + "/raw"
    if os.path.isdir(raw_directory) is False:
        os.makedirs(raw_directory)
        print "Created raw image directory at " + raw_directory

    segments_directory = parent_directory + "/segments"
    if os.path.isdir(segments_directory) is False:
        os.makedirs(segments_directory)
        print "Created segment image directory at " + segments_directory

    directory = 0
    while os.path.isdir(raw_directory + "/" + str(directory).zfill(5)):
        directory = directory + 1;

    # Package the directories where we are going to store our raw/segmented
    # images into a structure.
    full_directory_paths = list()
    full_directory_paths.append(raw_directory + "/" + str(directory).zfill(5))
    full_directory_paths.append(segments_directory + "/" + str(directory) \
            .zfill(5))

    if os.path.isdir(full_directory_paths[0]) is True \
        or os.path.isdir(full_directory_paths[1]) is True:
        print "Raw and segment directories are out of sync."
        sys.exit(1)

    os.makedirs(full_directory_paths[0])
    os.makedirs(full_directory_paths[1])
    print "Will store raw captured images in " + full_directory_paths[0]
    print "Will store segmented images in " + full_directory_paths[1]

    return full_directory_paths

def main():
    photos = deque()

    camera = photographer.Photographer()
    segment_generator = segmenter.Segmenter()

    full_directory_paths = form_directory_structure()

    thread.start_new_thread(camera.take_pictures, (full_directory_paths[0], \
        photos, ))
    thread.start_new_thread(segment_generator.use_deque_stream, ( \
            full_directory_paths[1], photos, ))

    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
