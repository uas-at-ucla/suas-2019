import cv2 as cv

# Use matplotlib without GUI
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt

OUTPUT_PATH = 'output'

# Save histogram to file
def showHistogram(hist):
	plt.plot(hist)
	plt.savefig(OUTPUT_PATH + '/histogram.png')

# Save binary image to file
def showThreshold(thresh):
	cv.imwrite(OUTPUT_PATH + '/thresh.jpg', thresh)

# Highlight and crop contour, then save and display
def showContour(img, contour):
	x, y, w, h = cv.boundingRect(contour)
	imgContour = cv.drawContours(img.copy(), [contour], 0, (255, 255, 255), 5)[y:y+h, x:x+w]
	cv.imwrite(OUTPUT_PATH + '/contour.png', imgContour)
	cv.imshow('Contour', imgContour)
	cv.waitKey()
