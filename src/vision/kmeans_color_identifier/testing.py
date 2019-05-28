from os import listdir, path, system

dir = 'images/test2/'

for image in listdir(dir):
    name = path.splitext(image)[0]
    extension = path.splitext(image)[1]
    filename = dir + name + extension
    
    if (extension == '.jpg'):
        print(filename)
        system("python3 identify_colors.py " + filename)

