import cv2 as cv
import numpy as np
from scipy import signal

from constants import *
from classify_target import *
from debug_output import *

import os
import sys

# Range threshold that handles cylindrical coordinates
def inRange_wrapHue(img, lower, upper):
    thresh = cv.inRange(img, lower, upper)
    hue_l, sat_l, val_l = lower
    hue_u, sat_u, val_u = upper

    if hue_l < 0:
        thresh += cv.inRange(img, (hue_l + 180, sat_l, val_l), (180, sat_u, val_u))
    elif hue_u > 180:
        thresh += cv.inRange(img, (0, sat_l, val_l), (hue_u - 180, sat_u, val_u))
    return thresh

def analyze_one_image(img_dir, img_id, out_dir, display=True, debug=True):
    img_path = os.path.join(img_dir, '%03d.jpg'%img_id)
    if debug:
        print(img_path)
    img = cv.imread(img_path)
    size = np.prod(img.shape[:2])

    # 1. Calculate histogram of just hue channel
    imgHLS = cv.cvtColor(img, cv.COLOR_BGR2HLS) # Hue-Lightness-Saturation
    histHue = cv.calcHist([imgHLS], [0], None, [180 // BIN_SIZE], [0,180]).ravel()
    showHistogram(img_id, histHue, out_dir=out_dir)

    # 2. Find relative maxima (most frequent hues)
    # Pad histogram with values from other end, to allow ends to be considered peaks
    peakIdxs = signal.argrelmax(np.pad(histHue, 1, 'wrap'))[0] - 1
    # Sort from most to least frequent hue
    peaks = sorted(peakIdxs, key=histHue.__getitem__, reverse=True)
    if debug:
        print('All hues:', list(peakIdxs * BIN_SIZE))

    # 3. Find best hue (and target) by trying to find a target with each one
    for pos in peaks:
        hue = int(pos) * BIN_SIZE
        thresh = inRange_wrapHue(imgHLS, range_lower(hue), range_upper(hue))
        showThreshold(img_id, thresh, out_dir=out_dir)
        # TODO: why does this happpen?
        if not np.count_nonzero(thresh):
            continue

        # Find biggest contour by pixel area
        _, contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        target = max(contours, key=cv.contourArea)
        area_px = cv.contourArea(target)
        area_gnd = area_px * px_to_gnd(ALTITUDE) ** 2

        # Human-readable logging
        summary = {
            'hue': hue,
            'contours': len(contours),
            'area_px': area_px,
            'area_gnd': area_gnd,
            'bounding_box': cv.boundingRect(target),
            'contour_detail': len(target)
        }

        # Filter by contour area, and by contour complexity
        if not GND_AREA_MIN < area_gnd < GND_AREA_MAX or len(contours) > DETAIL_MAX:
            if debug:
                print('Rejecting', summary)
            continue
        if debug:
            print('Accepting', summary)
        showContour(img_id, img, target, out_dir=out_dir, display=display)
        break
    else:
        if debug:
            print('No targets found')
        return None

    # 4. Classify the detected shapes, letters, and colors
    result = {
        'location': cv.boundingRect(target),
        'shape': classifyShape(target, display=display),
        'shapeColor': classifyColor(hue),
        'letter': None, # classifyLetter(None), # TODO
        'letterColor': None # classifyColor(None) # TODO
    }
    if debug:
        print('Target analysis:', result)
    return result

def analyze_image(img_path, display=True):
    img_dir = '/'.join(img_path.split('/')[:-1])
    img_id = int(img_path.split('/')[-1].split('.')[0])
    analyze_one_image(img_dir, img_id, 'output', display=display)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit('ERROR: no path given')
    img_path = sys.argv[1]
    analyze_image(img_path, display=True)

