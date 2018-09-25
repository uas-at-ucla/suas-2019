#!/bin/bash  

echo "Installing npm packages..."
cd src/ground/server
../tools/npm_install.sh
cd ../ui
../tools/npm_install.sh

echo "Doing Nothing... This script is a work in in progress"