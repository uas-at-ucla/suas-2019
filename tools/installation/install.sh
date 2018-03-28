#!/bin/sh

## Start timer of install script
start=`date +%s`;

##########################################################################
# Configuratons

echo "\nHello! Welcome to the UCLA UAS 2018 Software Installation!\n";

## Change to script directory so that the script can be called from anywhere.
cd "$(dirname "$0")";

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
        sudo apt-get install -qq curl;
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
# Update git submodules

cd ../../src/ground/
echo "Installing npm packages..."
git submodule init;
git submodule update --recursive;
cd client/;
npm install --loglevel=error;
cd ../../../tools/installation/;
echo "";

##########################################################################
# Install all Packages

## Check if all packages exist; if not, then install
if [ $OS = "Linux" ]
then
    echo "Checking all necessary packages..."
    echo "sudo apt-get install python3.5 python3-pip python-pip python3-dev build-essential\n"
    sudo apt-get install -qq python3.5 python3-pip python3-dev build-essential sl;
    sudo apt-get install -qq python-matplotlib python-numpy python2.7-dev sshpass;

    # Do Bazel stuff
    sudo add-apt-repository -y ppa:openjdk-r/ppa;
    sudo apt-get update && sudo apt-get -y -qq install openjdk-8-jdk;
    sudo apt-get -qq install libc++abi-dev;
    echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list;
    curl https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
    sudo apt-get update && sudo apt-get -y -qq install bazel clang-3.9 libc++-dev clang-format-3.5;
    sudo apt-get -y -qq install ruby;
    for i in $(seq 1 3);
    do
        sl;
    done
elif [ $OS = "Darwin" ]
then
    echo "Hi";
    ##### todo #####
    # Figure out bazel stuff here
    ################
fi

## See if pip version is 9.x -- If not, then update pip
version_pip=$(pip -V | grep -o "[0-9].[0-9].[0-9]" | grep -o "^[0-9]")
if [ $version_pip -gt 8 ]
then
    echo "Pip is greater than version 8.0.0\n";
else
    echo "Updating to latest pip version...";
    pip install --upgrade pip
fi

## Install python dependencies
RED='\033[0;31m';
NO_COLOR='\033[0m';
echo "Installing Python dependencies listed in ${RED}pip_requirements.txt${NO_COLOR}\nThis may take several seconds..."
sudo -H pip install -I -r pip_requirements.txt > /dev/null;
echo "Python dependencies installed.\n"


if [ $OS = "Linux" ]
then
    echo "Installing jMAVSim dependencies..."
    sudo apt-get -y -qq install ant;
fi

##########################################################################
# Do Gazebo stuff

if [ $OS = "Linux" ]
then
    echo "Installing Gazebo dependencies..."
    sudo apt-get -y -qq install libgazebo7-dev libopencv-dev libeigen3-dev protobuf-compiler liblz4-dev;
fi

#TODO(comran): Gazebo should be installed separately until we get it working
#              with Travis.

# if [ $OS = "Linux" ]
# then
#     ## Miscellaneous commands that we have to run
#     ## Note: not all commands may be needed
#     sudo add-apt-repository ppa:webupd8team/java -y;
#     sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
#     sudo apt-get update
#     sudo apt-get install -qq cmake python-empy;

#     ## Finally install gazebo stuff
#     echo "Installing gazebo...";
#     bash ubuntu_gazebo_install.sh;
# elif [ $OS = "Darwin" ]
# then
#     # MacOS comes with python
#     echo "Hi";
#     ##### todo #####
#     # Figure out Gazebo stuff here
#     ################
# fi

## Display time of installation
end=`date +%s`;
runtime=$((end-start));
echo "${NO_COLOR}\nTime of installation:\n${RED}$runtime seconds${NO_COLOR}\n"

echo "${NO_COLOR}\nIn order to test your installation, execute this command:\n${RED}./test-installation.sh${NO_COLOR}\n"

echo "${NO_COLOR}You can now run the ground control software by executing this command:\n${RED}sudo python ../../src/control/run.py${NO_COLOR}\n"
