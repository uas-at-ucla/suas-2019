import os
import cv2
import numpy as np
import argparse
from sklearn.cluster import KMeans

# our colors: might need tweaking
COLORS = [
    (255, 255, 255, 'white'),
    (35, 35, 35, 'black'),
    (220, 30, 30, 'red'),
    (23, 188, 11, 'green'),
    (55, 129, 205, 'blue'),
    (128, 128, 128, 'gray'),
    (245, 223, 20, 'yellow'),
	(108, 73, 155, 'purple'),
	(112, 64, 29, 'brown'),
	(242, 153, 19, 'orange')
]

def create_histogram(cluster):
    num_labels = np.arange(0, len(np.unique(cluster.labels_)) + 1)
    hist, _ = np.histogram(cluster.labels_, bins=num_labels)
    hist = hist.astype('float32')
    hist /= hist.sum()
    return hist

def extract_target(image):
    raw_image = image
    mask = np.zeros(image.shape[:2], np.uint8)
    bgdModel = np.zeros((1,65), np.float64)
    fgdModel = np.zeros((1,65), np.float64)

    h, w = image.shape[:2]    
    rect = (10, 10, w-60, h-60)

    cv2.grabCut(image, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
    applied_mask = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
    image = image * applied_mask[:,:, np.newaxis]
    image = cv2.bitwise_and(raw_image, image)
    return image


def distance(a, b):
    # 3D distance between two colors
    return ((a[0] - b[0])** 2 + (a[1] - b[1])** 2 + (a[2] - b[2])** 2)
    
def name_color(rgb):
    # compare a color with our (rgb to name) dictionary
    closest_match = min(COLORS, key=lambda x: distance(rgb, x))
    return closest_match[3]


# START
parser = argparse.ArgumentParser()
parser.add_argument("image_path")
args = parser.parse_args()

image_path = args.image_path
raw_image = cv2.imread(image_path)
height, width, _ = np.shape(raw_image)

# use a foreground extraction tool to mask everything around the target
image = extract_target(raw_image)
# save a copy of this stage cause it looks cool
masked_image = image
# reshape the image to be a simple list of RGB pixels
image = image.reshape((height * width, 3))

# pick the three most common colors (one of which will be the black mask)
clt = KMeans(n_clusters=3)
clt.fit(image)
# remove pure black (the mask) from the clusters
clt.cluster_centers_ = clt.cluster_centers_.astype(int)
clt.cluster_centers_ = clt.cluster_centers_[~np.all(clt.cluster_centers_ == 0, axis=1)]
# create a histogram of color frequency
histogram = create_histogram(clt)

# remove the most common color (should be the black mask), but this is kinda jank
max_index = np.where(histogram == np.amax(histogram))
histogram = np.delete(histogram, max_index)
print(histogram)
# associate the histogram results with the cluster centers
combined = zip(histogram, clt.cluster_centers_)
# sort by historgram value (most dominant color)
combined = sorted(combined, key=lambda x: x[0])

for index, rows in enumerate(combined):
    color = rows[1]
    red, green, blue = int(color[2]), int(color[1]), int(color[0])
    rgb = (red, green, blue)

    name = name_color(rgb)
    if index:
        print("Shape Color: {}, {}".format(name, rgb))
    else:
        print("Letter Color: {}, {}".format(name, rgb))

# Really hacky way to save images
os.makedirs('output', exist_ok=True)
path = os.path.join('output', image_path.split('/')[-1])
cv2.imwrite(path, masked_image)

# cv2.namedWindow('img', cv2.WINDOW_NORMAL)
cv2.namedWindow('masked', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('img', 500, 500)
cv2.resizeWindow('masked', 500, 500)

# cv2.imshow('img', raw_image)
cv2.imshow('masked', masked_image)
cv2.waitKey(2000)



