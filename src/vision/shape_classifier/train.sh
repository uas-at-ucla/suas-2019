#!/bin/bash

# Change to script directory.
cd "$(dirname "$0")"

# Generate targets to train with.
# TODO(comran): Don't use the old image generator.

# Now to do the training...
cd darkflow

mkdir bin
cd bin
# Download the weights, if they weren't downloaded already.
wget -nc https://pjreddie.com/media/files/tiny-yolo-voc.weights
cd ..

# Train!
flow --train --model ./cfg/yolo-auvsi.cfg --dataset /home/comran/Code/suas_2018_vision_training_set/real/ --annotation /home/comran/Code/suas_2018_vision_training_set/real/ --gpu 1.0
