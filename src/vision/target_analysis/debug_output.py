import cv2 as cv
import matplotlib.pyplot as plt

__all__ = ['showHistogram', 'showThreshold', 'showContour']

OUTPUT_PATH = 'output'

# Save histogram to file
def showHistogram(img_id, hist, out_dir=OUTPUT_PATH):
    fig, ax = plt.subplots()
    ax.plot(hist)
    fig.savefig(OUTPUT_PATH + '/histogram%03d.png'%img_id)
    plt.close(fig)

# Save binary image to file
def showThreshold(img_id, thresh, out_dir=OUTPUT_PATH):
    cv.imwrite(OUTPUT_PATH + '/thresh%03d.jpg'%img_id, thresh)

# Highlight and crop contour, then save and display
def showContour(img_id, img, contour, out_dir=OUTPUT_PATH, display=False):
    x, y, w, h = cv.boundingRect(contour)
    imgContour = cv.drawContours(img.copy(), [contour], 0, (255, 255, 255), 5)[y:y+h, x:x+w]
    cv.imwrite(OUTPUT_PATH + '/contour%03d.png'%img_id, imgContour)
    if display:
        cv.imshow('Contour', imgContour)
        cv.waitKey(2000)


