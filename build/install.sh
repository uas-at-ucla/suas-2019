#!/bin/sh

##########################################################################
# Configuratons

echo "\nHello! Welcome to the UCLA UAS 2018 Software Installation!\n";

## Change to script directory so that the script can be called from anywhere.
cd "$(dirname "$0")"

## Determine operating system
OS=$(uname -s)

if [ $OS = "Darwin" ]
then
    if [ "xcode-select --install 2>&1 | grep installed" ]
    then
        : # Xcode command line tools are installed
    else
      echo "Use the prompt to install the Xcode command line tools (needed for Homebrew), and then run this script again.";
      exit 1;
    fi
    which -s brew
    if [[ $? != 0 ]]
    then
        # Install Homebrew
        /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
    else
        brew update
    fi
fi

##########################################################################
# Install Node.js

## See if node version is 8.x -- If not, then update node
version_node=$(node -v | grep -o "[0-9].[0-9].[0-9]" | grep -o "^[0-9]")
if [ $version_node -gt 7 ]
then
    echo "Node.js is greater than version 7.0.0\n";
else
    if [ $OS = "Linux" ]
    then
        echo "Installing Node.js...";
        curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -;
        sudo apt-get install -y nodejs;
    elif [ $OS = "Darwin" ]
    then
        echo "Installing Node.js...";
        brew install node;
    else
        echo "Please install Node.js first, and then run this script again."
        exit 1;
    fi
    echo "\n\n";
fi

##########################################################################
# Install all Python dependencies

## Check if all Python packages exist; if not, then install
if [ $OS = "Linux" ]
then
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
elif [ $OS = "Darwin" ]
then
    # MacOS comes with python
    which -s pip
    if [[ $? != 0 ]]
    then
        sudo easy_install pip;
    fi
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
echo "Installing Python dependencies listed in ${RED}build/pip_requirements.txt${NO_COLOR}\nThis may take several seconds..."
sudo pip install --upgrade pip
sudo -H pip install -I -r pip_requirements.txt;
echo "Python dependencies installed.\n"

##########################################################################
# Update git submodules
echo "Installing npm packages..."
git submodule init;
git submodule update --recursive;
cd ../ground/client;
npm install --loglevel=error;
echo "";

##########################################################################
# Do docker stuff
if [ $OS = "Linux" ]
then
    docker_exists=$(groups | grep -o "docker");
    if [ $docker_exists = "docker" ]
    then
        echo "A ${RED}docker${NO_COLOR} group already exists.\nIf the docker is not working, remove the group by executing the command ${RED}sudo groupdel docker${NO_COLOR} and run this installation script again.\n";
    else
        echo "\nNo group ${RED}docker${NO_COLOR} exists. Setting up docker..."
        sudo groupadd docker;
        sudo usermod -aG docker $USER;
        echo "You must reboot your machine and run these final two commands (please keep note of them after reboot) in order to complete installation:\n${RED}docker pull auvsisuas/interop-server\nsudo ../ground/interop/tools/setup_docker.sh\n";
    fi
elif [ $OS = "Darwin" ]
then
    which -s docker
    if [[ $? != 0 ]]
    then
        brew cask install docker
    fi
    open -a Docker
    docker pull auvsisuas/interop-server
fi

echo "${NO_COLOR}After installation is complete, you can run the ground control software by executing this command:\n${RED}sudo python ../ground/run_ground.py${NO_COLOR}\n"
