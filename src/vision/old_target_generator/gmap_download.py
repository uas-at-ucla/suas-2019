import os
import sys
import json
import urllib
import random
import time


def main():
    if len(sys.argv) != 2:
        print("Expected filename.")
        sys.exit(1)

    print("Opening images to download from " + sys.argv[1])
    opened_json = open(sys.argv[1])

    data = json.load(opened_json)

    for img_to_download in data:
        url = img_to_download['url']
        urllib.urlretrieve(
            url, "DATA/gmap_tiles/" + url.split('/')[-1] \
                    .replace("&", "_") \
                    .replace("?", "-") \
                    .replace("=", "__") \
                    + ".jpg")
        print(img_to_download['url'])

if __name__ == '__main__':
    main()
