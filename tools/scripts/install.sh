#!/bin/bash

PACKAGES="docker python2.7 tmux git"
OPTIONAL_PACKAGES="node"
MACOS_PACKAGES="brew docker-machine docker-machine-nfs virtualbox"
INSTALL_REQUIRED="false"
NEED_TO_INSTALL=""
ACTION_REQUIRED=""

PLATFORM=$(uname -s)

function check_if_installed {
  which $1 > /dev/null 2>&1
  return $?
}

function install_package {
  check_if_installed "$1"
  if [ $? -ne 0 ]
  then
    if [ "$PLATFORM" == "Darwin" ]
    then
      brew install $1
    else
      sudo apt-get install -y $1
    fi
  fi
}

if [ "$EUID" == 0 ]
  then echo "Don't run as root."
  exit 1
fi

if [ "$PLATFORM" == "Darwin" ]
then
  PACKAGES="$PACKAGES $MACOS_PACKAGES"
fi

if [ "$PLATFORM" != "Darwin" ] && \
   [ "$PLATFORM" != "Linux"  ]
then
  echo "Unsupported platform: $PLATFORM"
  exit 1
fi

for PACKAGE in $PACKAGES; do
  check_if_installed "$PACKAGE"

  if [ $? -ne 0 ]
  then
    INSTALL_REQUIRED="true"
    NEED_TO_INSTALL="$NEED_TO_INSTALL $PACKAGE"
  fi
done

if [ "$INSTALL_REQUIRED" == "false" ]
then
  echo "All necessary UAS@UCLA host packages are installed."
  exit 0
fi

echo "Need to install$NEED_TO_INSTALL"

sudo echo "Checking if sudo privileges work..." > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo "Please run './tools/scripts/install.sh' directly to install dependencies."
  exit 1
fi

PACKAGES="$OPTIONAL_PACKAGES $PACKAGES"


if [ "$PLATFORM" == "Darwin" ]
then
  check_if_installed "brew"
  if [ $? -ne 0 ]
  then
    /usr/bin/ruby -e "$(curl -fsSL \
      https://raw.githubusercontent.com/Homebrew/install/master/install)"

    if [ $? -ne 0 ]
    then
      echo "Error installing Homebrew."
      exit 1
    fi
  fi

  check_if_installed "docker-machine"
  if [ $? -ne 0 ]
  then
    brew install docker-machine
  fi

  check_if_installed "docker-machine-nfs"
  if [ $? -ne 0 ]
  then
    brew install docker-machine-nfs
  fi

  check_if_installed "virtualbox"
  if [ $? -ne 0 ]
  then
    # brew cask install virtualbox (requires sudo)
    ACTION_REQUIRED="$ACTION_REQUIRED""Please install VirtualBox from https://www.virtualbox.org\n"
  fi
fi

check_if_installed "docker"
if [ $? -ne 0 ]
then
  if [ "$PLATFORM" == "Darwin" ]
  then
    brew install docker
  else
    sudo apt-get install -y \
     apt-transport-https \
     ca-certificates \
     curl \
     gnupg2 \
     software-properties-common

    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

    sudo apt-key fingerprint 0EBFCD88

    ## Does not have a bionic file
    ## Note the linux/debian, not linux/ubuntu
    sudo add-apt-repository \
      "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) \
      stable"

    sudo apt-get update
    sudo apt-get install -y docker-ce

    sudo groupadd -a -G docker $USER
  fi
fi

check_if_installed "python2.7"
if [ $? -ne 0 ]
then
  if [ "$PLATFORM" == "Darwin" ]
  then
    brew install python@2
  else
    sudo apt-get install -y python2.7
  fi
fi

install_package "tmux"
install_package "git"
install_package "nodejs"

if [ "$ACTION_REQUIRED" != "" ]
then
  echo ""
  printf "\033[91mAction Required:\033[0m\n"
  printf "$ACTION_REQUIRED\n"
  exit 1
fi
