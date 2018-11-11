# AUVSI ADLC Training Samples Creation.

[Forked from the TAS team](https://github.com/amitibo/auvsi-targets)

## Installation

### Linux
    > git clone https://github.com/amitibo/auvsi-targets.git
    > cd auvsi_targets
    > sudo apt-get install python-opencv virtualenvwrapper ttf-dejavu
    > mkvirtual --system-site-packages auvsi # we need access to opencv from the virtualenv
    (auvsi) > pip install -r requirements.txt
    (auvsi) > ./install_aggdraw

To install do:

    > git clone https://github.com/amitibo/auvsi-targets.git
    > cd auvsi_targets
    > pip install -r requirements.txt

## How to Use

* To create shape trainig samples run ```./create_patches.py```
* To create letter training samples run ```./create_letter_samples.py```

Input images are under ```DATA/resized_images``` and ```DATA/renamed_images```, flight data is under ```DATA/flight_data```.

The shape samples and letter samples are created under ```DATA/train_images```
and ```DATA/train_letter``` respectively. Each sample is made of an image
file and a corresponding label file.
