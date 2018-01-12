#!/bin/sh

# todo: should automatically install docker for us
# Future Expansion: Get all python flask installations automatically setup here

echo "\nHello! Welcome to the UAS 2018 Installation!\n";


# Install Node.js

## See if node is version 8.x -- If not, then update node
version=$(node -v | grep -o "[0-9].[0-9].[0-9]" | grep -o "^[0-9]")
if [ $version -gt 7 ]
then
    echo "Node.js is greater than version 7.0.0\n\n";
else
    curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -;
    sudo apt-get install -y nodejs;
fi





# Update git submodules
git submodule init &&
    git submodule update --recursive &&
    cd client &&
    npm install &&
    cd ..;

