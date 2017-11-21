import os
from PIL import Image
import cPickle as pickle
rootdir = '/home/uas/auvsi-targets/DATA/test_letter'
extension = '.txt'

labels = []
for subdir, dirs, files in os.walk(rootdir):
    for file in sorted(files):
        ext = os.path.splitext(file)[-1].lower()
        if ext == extension:
        	with open(os.path.join(subdir, file)) as f:
		    for line in f:
		        line = line.split() # to deal with blank 
		        if line:            # lines (ie skip them)
		            for i in line:
		            	line = int(i)
		            labels.append(line)

pickle.dump(labels, open("test_labels.p", "wb"))
