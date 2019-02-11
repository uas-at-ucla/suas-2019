import urllib
import requests
import os
import json
import sys

#Use python ./googleMapsDownloader

def downloadImage(mag, image_list = [], *args):
    for download_data in image_list:
        x = str(download_data['x'])
        y = str(download_data['y'])
        image_name = 'mag-' + mag + '_x-' + x + '_y-' + y + '.jpg'
        if not os.path.isfile(image_name):
            url = download_data['url']
            r = requests.get(url) # create HTTP response object 
            # send a HTTP request to the server and save 
            # the HTTP response in a response object called r 
            with open(image_name, 'wb') as f: 
                # Saving received content as a png file in 
                # binary format 
                # write the contents of the response (r.content) 
                # to a new file in binary mode. 
                f.write(r.content)
                print(image_name + '/n')
        

def main():
    os.chdir(os.path.dirname(sys.argv[0]))
    current_directory = os.getcwd()
    json_file = open('tileUrls.json')
    json_str = json_file.read()
    json_data = json.loads(json_str)
    download_destination = str("googleMapsImages")
    if not os.path.exists(os.path.join(current_directory, download_destination)):
            os.makedirs(download_destination)
    os.chdir("./" + download_destination)
    for magnification, image_list in json_data.items():
        current_directory = os.getcwd()
        tmp_path = os.path.join(current_directory, magnification)
        if not os.path.exists(tmp_path):
            os.makedirs(tmp_path)
        os.chdir(tmp_path)
        mag = str(magnification)
        downloadImage(mag, image_list)
        os.chdir("../")
    os.chdir("../")
    print(current_directory)

if (__name__ == "__main__"):
    main()
