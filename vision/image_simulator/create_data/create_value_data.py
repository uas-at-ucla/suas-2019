import os
from PIL import Image
import cPickle as pickle
rootdir = '/home/uas/auvsi-targets/DATA/test_letter'
extension = '.jpg'

imageValue = []
for subdir, dirs, files in os.walk(rootdir):
    for file in sorted(files):
        ext = os.path.splitext(file)[-1].lower()
        if ext == extension:
        	img = Image.open(os.path.join(subdir, file)).convert('L')
		WIDTH, HEIGHT = img.size
		data = list(img.getdata())
		imageValue.append(data)

pickle.dump(imageValue, open("test_values.p", "wb"))
