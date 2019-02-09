import cv2 as cv
import numpy as np
from scipy import signal

from classify_target import *
from debug_output import *

import sys

# Histogram bin size (must divide 180)
BIN_SIZE = 4

# +/- degrees to allow in hue threshold
HUE_MARGIN = 10

# Valid contour area range (as percentage of image size)
CAREA_MIN = 0.00005
CAREA_MAX = 0.05

def range_lower(hue):
	return (hue - HUE_MARGIN, 0, 32)

def range_upper(hue):
	return (hue + HUE_MARGIN, 255, 255)

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

if len(sys.argv) < 2:
	sys.exit("ERROR: no path given")

img = cv.imread(sys.argv[1])
size = np.prod(img.shape[:2])

# 1. Calculate histogram of just hue channel
imgHSV = cv.cvtColor(img, cv.COLOR_BGR2HLS) # Hue-Lightness-Saturation
histHue = cv.calcHist([imgHSV], [0], None, [180 // BIN_SIZE], [0,180]).ravel()

showHistogram(histHue)

# 2. Find relative maxima (most frequent hues)
# Pad histogram with values from other end, to allow ends to be considered peaks
peakIdxs = signal.argrelmax(np.pad(histHue, 1, 'wrap'))[0] - 1
# Sort from most to least frequent hue
peaks = sorted(zip(histHue[peakIdxs], peakIdxs), reverse=True)

print("All hues:", list(peakIdxs * BIN_SIZE))

# 3. Find best hue (and target) by trying to find a target with each one
for count,pos in peaks:
	hue = int(pos) * BIN_SIZE

	thresh = inRange_wrapHue(imgHSV, range_lower(hue), range_upper(hue))
	showThreshold(thresh)

	# TODO: why does this happpen?
	if not np.count_nonzero(thresh):
		continue

	# Find biggest contour by pixel area
	contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	target, area = max(zip(contours, map(cv.contourArea, contours)), key = lambda p: p[1])

	# Human-readable logging
	summary = {
		'hue': hue,
		'contours': len(contours),
		'pixel_ratio': count / size,
		'area_ratio': area / size,
		'bounding_box': cv.boundingRect(target),
		'contour_detail': len(target)
	}

	# Filter by contour area to image size ratio
	if not CAREA_MIN < area / size < CAREA_MAX:
		print("Rejecting", summary)
		continue

	print("Accepting", summary)
	showContour(img, target)
	break
else:
	sys.exit("No targets found")

# 4. Classify the detected shapes, letters, and colors
result = {
	'location': cv.boundingRect(target),
	'shape': classifyShape(target),
	'shapeColor': classifyColor(hue),
	'letter': classifyLetter(None), # TODO
	'letterColor': classifyColor(None) # TODO
}

print("Target analysis:", result)
