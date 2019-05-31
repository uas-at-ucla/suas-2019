import os
import cv2 as cv
import matplotlib.pyplot as plt

__all__ = ['showHistogram', 'showThreshold', 'showContour']

OUTPUT_PATH = 'output'

# Save histogram to file
def showHistogram(img_id, hist, out_dir=OUTPUT_PATH):
    fig, ax = plt.subplots()
    ax.plot(hist)
    path = os.path.join(out_dir, 'histograms/histogram%03d.png'%img_id)
    fig.savefig(path)
    plt.close(fig)

# Save binary image to file
def showThreshold(img_id, thresh, out_dir=OUTPUT_PATH):
    path = os.path.join(out_dir, 'thresholds/thresh%03d.jpg'%img_id)
    cv.imwrite(path, thresh)

# Highlight and crop contour, then save and display
def showContour(img_id, img, contour, out_dir=OUTPUT_PATH, display=False):
    x, y, w, h = cv.boundingRect(contour)
    imgContour = cv.drawContours(img.copy(), [contour], 0, (255, 255, 255), 5)[y:y+h, x:x+w]
    path = os.path.join(out_dir, 'countours/contour%03d.png'%img_id)
    cv.imwrite(path, imgContour)
    if display:
        cv.imshow('Contour', imgContour)
        cv.waitKey(2000)


