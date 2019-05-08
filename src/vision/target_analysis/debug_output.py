import os
import cv2 as cv
import matplotlib.pyplot as plt

__all__ = ['showHistogram', 'showThreshold', 'showContour']

OUTPUT_PATH = 'output'
os.makedirs(OUTPUT_PATH, exist_ok=True)

# Save histogram to file
def showHistogram(hist):
    fig, ax = plt.subplots()
    ax.plot(hist)
    fig.savefig(OUTPUT_PATH + '/histogram.png')
    plt.close(fig)

# Save binary image to file
def showThreshold(thresh):
    cv.imwrite(OUTPUT_PATH + '/thresh.jpg', thresh)

# Highlight and crop contour, then save and display
def showContour(img, contour):
    x, y, w, h = cv.boundingRect(contour)
    imgContour = cv.drawContours(img.copy(), [contour], 0, (255, 255, 255), 5)[y:y+h, x:x+w]
    cv.imwrite(OUTPUT_PATH + '/contour.png', imgContour)
    cv.imshow('Contour', imgContour)
    cv.waitKey(200)
