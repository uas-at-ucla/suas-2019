#!/bin/bash

unset PACKAGES
unset MACOS_PACKAGES
unset INSTALL_REQUIRED
unset PLATFORM
unset NEED_TO_INSTALL

PACKAGES="docker python2.7 tmux git"
MACOS_PACKAGES="brew docker-machine-nfs"
INSTALL_REQUIRED="false"
NEED_TO_INSTALL=""

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

echo "Need to install $NEED_TO_INSTALL"

if [ "$PLATFORM" == "Darwin" ]
then
  check_if_installed "brew"
  if [ $? -ne 0 ]
  then
    /usr/bin/ruby -e "$(curl -fsSL \
      https://raw.githubusercontent.com/Homebrew/install/master/install)"

    if [ $? -ne 0 ]
    then
      echo "Error installing brew."
    fi
  fi

  check_if_installed "docker-machine-nfs"
  if [ $? -ne 0 ]
  then
    curl -s "https://raw.githubusercontent.com/adlogix/docker-machine-nfs/" \
      "master/docker-machine-nfs.sh" | sudo tee \
      /usr/local/bin/docker-machine-nfs > /dev/null && sudo chmod +x \
      /usr/local/bin/docker-machine-nfs
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

    curl -fsSL https://download.docker.com/linux/debian/gpg | sudo apt-key add -

    sudo apt-key fingerprint 0EBFCD88

    sudo add-apt-repository \
      "deb [arch=amd64] https://download.docker.com/linux/debian \
      $(lsb_release -cs) \
      stable"

    sudo apt-get update
    sudo apt-get install -y docker-ce
  fi
fi

check_if_installed "$1"
if [ $? -ne 0 ]
then
  if [ "$PLATFORM" == "Darwin" ]
  then
    brew install python
  else
    sudo apt-get install -y python2.7
  fi
fi

install_package "tmux git"
