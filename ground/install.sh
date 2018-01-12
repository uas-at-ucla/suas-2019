#!/bin/sh

# todo: should automatically install docker for us
# Future Expansion: Get all python flask installations automatically setup here

# Install Node.js
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -;
sudo apt-get install -y nodejs;

# Update git submodules
git submodule init &&
    git submodule update --recursive &&
    cd client &&
    npm install &&
    cd ..;

