import os
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

from constants import *

__all__ = ['showHistogram', 'showThreshold', 'showContour']

OUTPUT_PATH = 'output'

# Save histogram to file
def showHistogram(img_id, hist, out_dir=OUTPUT_PATH):
    fig, ax = plt.subplots()
    ax.plot(hist)
    path = os.path.join(out_dir, ('histograms/histogram%0' + str(N_DIGITS) + 'd.png')%img_id)
    fig.savefig(path)
    plt.close(fig)

# Save binary image to file
def showThreshold(img_id, thresh, out_dir=OUTPUT_PATH):
    path = os.path.join(out_dir, ('thresholds/thresh%0' + str(N_DIGITS) + 'd.jpg')%img_id)
    cv.imwrite(path, thresh)

# Highlight and crop contour, then save and display
def showContour(img_id, img, contour, out_dir=OUTPUT_PATH, display=False):
    x, y, w, h = cv.boundingRect(contour)
    # imgContour = cv.drawContours(img.copy(), [contour], 0, (255, 255, 255), 5)[y:y+h, x:x+w]
    mask = np.zeros(img.shape[:2], dtype='uint8') * 255
    mask = cv.drawContours(mask, [contour], 0, 
        color=1, thickness=-1)[y:y+h, x:x+w]
    img = img[y:y+h, x:x+w]
    imgContour = cv.bitwise_and(img, img, mask=mask)
    path = os.path.join(out_dir, ('countours/contour%0' + str(N_DIGITS) + 'd.png')%img_id)
    cv.imwrite(path, imgContour)
    if display:
        cv.imshow('Contour', imgContour)
        cv.waitKey(2000)


