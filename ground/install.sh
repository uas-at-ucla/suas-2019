#!/bin/sh

echo "\nHello! Welcome to the UCLA UAS 2018 Ground Software Installation!\n";

##########################################################################
# Install Node.js

## See if node version is 8.x -- If not, then update node
version_node=$(node -v | grep -o "[0-9].[0-9].[0-9]" | grep -o "^[0-9]")
if [ $version_node -gt 7 ]
then
    echo "Node.js is greater than version 7.0.0\n";
else
    echo "Installing Node.js...";
    curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -;
    sudo apt-get install -y nodejs;
    echo "\n\n";
fi

##########################################################################
# Install all Python dependencies

## Check if all Python packages exist; if not, then install
if [ $(dpkg-query -W -f='${Status}' python3.5 2>/dev/null | grep -c "ok installed") -eq 0 ]
then
    sudo apt-get install python3.5;
fi
if [ $(dpkg-query -W -f='${Status}' python3-pip 2>/dev/null | grep -c "ok installed") -eq 0 ]
then
    sudo apt-get install python3-pip;
fi
if [ $(dpkg-query -W -f='${Status}' python3-dev 2>/dev/null | grep -c "ok installed") -eq 0 ]
then
    sudo apt-get install python3-dev;
fi
if [ $(dpkg-query -W -f='${Status}' build-essential 2>/dev/null | grep -c "ok installed") -eq 0 ]
then
    sudo apt-get install build-essential;
fi

## See if pip version is 9.x -- If not, then update pip
version_pip=$(pip -V | grep -o "[0-9].[0-9].[0-9]" | grep -o "^[0-9]")
if [ $version_pip -gt 8 ]
then
    echo "Pip is greater than version 8.0.0\n";
else
    echo "Warning! The version of pip is $version_pip\nConsider updating to the latest version of pip.";

fi

## Install python dependencies
RED='\033[0;31m';
NO_COLOR='\033[0m';
echo "Installing Python dependencies listed in ${RED}/../build/pip_requirements.txt${NO_COLOR}\nThis may take several seconds..."
pip install --quiet -r ../build/pip_requirements.txt;
pip install --quiet socketio-client docker;
echo "Python dependencies installed.\n"

##########################################################################
# Update git submodules
echo "Installing npm packages..."
git submodule init;
git submodule update --recursive;
cd client;
npm install --loglevel=error;
echo "\n";
cd ..;

##########################################################################
# Do docker stuff
docker_exists=$(groups | grep -o "docker");
if [ $docker_exists = "docker" ]
then
    echo "A ${RED}docker${NO_COLOR} group already exists.\nIf the docker is not working, remove the group by executing the command ${RED}sudo groupdel docker${NO_COLOR} and run this installation script again.\n";
else
    echo "\nNo group ${RED}docker${NO_COLOR} exists. Setting up docker..."
    sudo groupadd docker;
    sudo usermod -aG docker $USER;
    echo "You must reboot your machine and run these final two commands (please keep note of them after reboot) in order to complete installation:\n${RED}docker pull auvsisuas/interop-server\nsudo ./interop/tools/setup_docker.sh\n";
fi

echo "${NO_COLOR}After installation is complete, you can run the ground control software by executing this command:\n${RED}sudo python ../control/run.py${NO_COLOR}\n"