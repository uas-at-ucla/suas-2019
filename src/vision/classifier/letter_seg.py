import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import sys

from constants import *

# Really stupid print function; used to force numpy to print the entire array
def print_full(arr):
    for x in arr:
        for y in x:
            print(y, end=' ')
        print('', end='\n')

def letter_seg(path):
    # Convert image to a numpy array
    img = Image.open(path)
    img = img.resize((IMG_SIZE,IMG_SIZE))
    img = np.array(img).astype(np.float32)
    Z = img.reshape(IMG_SIZE*IMG_SIZE,3)

    # Use k-means to convert image to 2 colors
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 4
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    label = label[:,0].reshape(IMG_SIZE, IMG_SIZE)

    # Get rid of the background
    masks = []
    for i in range(K):
        if label[0,0] == i:
            continue
        masks.append((label == i).astype(np.uint8))
    
    # Get rid of the noise (will have the most complicated contourset)
    cset = []
    for i in range(3):
        _, contours, hierarchy = cv2.findContours(masks[i], cv2.RETR_EXTERNAL, 
                                                  cv2.CHAIN_APPROX_NONE)
        cset.append(contours)
    noise_i = cset.index(max(cset, key=len))
    masks.pop(noise_i)

    # Merge the letter mask into the shape mask
    shape = max(masks, key=np.sum)
    letter = min(masks, key=np.sum)
    shape = cv2.bitwise_or(shape, letter, mask=None).astype(np.uint8)
    masks = [shape, letter]

    # Keep the largets connected component in each
    for i in range(2):
        n, labels, stats, centroids = cv2.connectedComponentsWithStats(masks[i])
        max_area = 0
        for c in range(n):
            if stats[c][cv2.CC_STAT_LEFT] == 0 and stats[c][cv2.CC_STAT_TOP] == 0:
                continue
            if stats[c][cv2.CC_STAT_AREA] > max_area:
                max_area = stats[c][cv2.CC_STAT_AREA]
                max_index = c
        masks[i] = (labels == max_index).astype(np.uint8)

    # Fill in gaps in the shape contour (due to removing noise)
    _, contours, hierarchy = cv2.findContours(masks[0], cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_TC89_L1)
    masks[0] = cv2.drawContours(masks[0], contours, 0, 
        color=1, thickness=-1)
    return masks[0], masks[1]

# def letter_seg(path):
#     #convert image to a numpy array
#     img = Image.open(path)
#     img = img.resize((IMG_SIZE,IMG_SIZE))
#     img = np.array(img).astype(np.float32)
#     Z = img.reshape(IMG_SIZE*IMG_SIZE,3)
#     #use k-means to convert image to 2 colors
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
#     K = 4
#     ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
#     for x in label[:,0].reshape(IMG_SIZE, IMG_SIZE):
#         for y in x:
#             print(y, end=' ')
#         print('', end='\n')
#     center = np.uint8(center)
#     res = center[label.flatten()]
#     res2 = res.reshape((img.shape))
#     #convert the image to black and white
#     res2 = (res2 == res2[0,0]).astype(np.float32)
#     #create mask for floodfilling
#     h, w = res2.shape[:2]
#     mask = np.zeros((h+2, w+2), np.uint8)
#     #meant to close gaps but doesn't work well
#     #kernel = np.ones((14,14),np.uint8)
#     #res2 = cv2.morphologyEx(res2, cv2.MORPH_CLOSE, kernel)
#     #Floodfill from point (0, 0)
#     cv2.floodFill(res2, mask, (0,0), 0)
#     return np.expand_dims(res2[:,:,0], axis=2)

