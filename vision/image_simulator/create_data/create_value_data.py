import os
from PIL import Image
import cPickle as pickle
rootdir = '' #path to directory of letter images in auvsi-targets
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

pickle.dump(imageValue, open("train_values.p", "wb"))
