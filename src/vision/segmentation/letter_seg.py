import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import sys

def letter_seg():
    #get path
    path = sys.argv[1]

    #convert image to a numpy array
    img = Image.open(path)
    img = img.resize((224,224))
    img = np.array(img).astype(np.float32)
    Z = img.reshape(224*224,3)

    #use k-means to convert image to 2 colors
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 2
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((img.shape))

    #convert the image to black and white
    res2 = (res2 == res2[0,0]).astype(np.float32)

    #create mask for floodfilling
    h, w = res2.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)

    #meant to close gaps but doesn't work well
    #kernel = np.ones((14,14),np.uint8)
    #res2 = cv2.morphologyEx(res2, cv2.MORPH_CLOSE, kernel)

    #Floodfill from point (0, 0)
    cv2.floodFill(res2, mask, (0,0), 0)
    
    return res2

