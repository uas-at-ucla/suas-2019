#!/bin/bash

TARGET_LABEL="$1"
TARGET_VERSION="$2"
TARGET_PUBLISH="$3"

STAGING="$(pwd)/build/staging"
STARTDIR=$(pwd)

if [[ $OSTYPE == darwin* ]]; then
	# OSX
	echo 'os: osx'

	# brew tap homebrew/versions
	# brew install gcc48 gawk
	# sudo easy_install pip
	# sudo pip install awscli virtualenv

	alias gawk=gcc
	MAKEARGS="NATIVE_CXX=g++-4.8 NATIVE_CC=gcc-4.8 NATIVE_AS=gcc-4.8 NATIVE_LD=g++-4.8"
	OSID="osx"
	AWSCMD="aws"
	BINNAME="apm"
elif [[ $OSTYPE == cygwin* ]]; then
	# WIN
	echo 'os: win'

	# pip install awscli virtualenv

	MAKEARGS=""
	OSID="win"
	AWSCMD="aws.cmd"
	BINNAME="apm.exe"
else
	# LINUX
	echo 'os: linux'

	# sudo apt-get install -y gcc-arm-linux-gnueabi make gawk gcc-4.8 g++-4.8 python-dev
	# sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 50
	# sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 50
	# sudo pip install awscli virtualenv

	alias gawk=gcc
	MAKEARGS=""
	OSID="linux"
	AWSCMD="aws"
	BINNAME="apm"
fi

# Define params file
if [[ $TARGET_LABEL == rover ]]; then
	PARMS='Tools/autotest/Rover.parm'
	TARGET_ARDU='APMrover2'
elif [[ $TARGET_LABEL == plane ]]; then
	PARMS='Tools/autotest/ArduPlane.parm'
	TARGET_ARDU='ArduPlane'
else
	PARMS='Tools/autotest/copter_params.parm'
	TARGET_ARDU='ArduCopter'
fi

echo "Building in $(pwd) ..."

echo "label: $TARGET_LABEL"
echo "classname: $TARGET_ARDU"
echo "version: $TARGET_VERSION"

echo "Downloading https://github.com/tcr3dr/ardupilot-releases/archive/builder-$TARGET_LABEL-$TARGET_VERSION.tar.gz"

set -e
set -x

rm -rf build
mkdir -p build/ardupilot
mkdir -p build/test
mkdir -p build/out
mkdir -p publish
cd build
git clone https://github.com/dronekit/ardupilot-releases -b builder-$TARGET_LABEL-$TARGET_VERSION ardupilot --depth 10
cd ardupilot/$TARGET_ARDU

buildit () {
  make clean BUILDROOT=$STAGING || true
  make configure SKETCHBOOK=$(pwd)/.. BUILDROOT=$STAGING || true
  make sitl-mount SKETCHBOOK=$(pwd)/.. BUILDROOT=$STAGING -j64 $MAKEARGS
}

# Thrice is nice. Works around build bugs in ArduCopter 3.2.x
buildit || buildit || buildit

cp $STARTDIR/build/ardupilot/$TARGET_ARDU/$TARGET_ARDU.elf $STARTDIR/build/out/$BINNAME
strip $STARTDIR/build/out/$BINNAME

# Parameters
cp $STARTDIR/build/ardupilot/$PARMS $STARTDIR/build/out/default.parm

# Windows needs Cygwin DLLs packaged
if [[ $OSTYPE == cygwin* ]]; then
	cp c:/cygwin/bin/cyggcc_s-1.dll c:/cygwin/bin/cygstdc++-6.dll c:/cygwin/bin/cygwin1.dll $STARTDIR/build/out
fi

(
	set -e

	cd $STARTDIR
	if [[ ! -d env ]]; then
		virtualenv env
	fi
	if [[ $OSTYPE == cygwin* ]]; then
		source ./env/Scripts/activate
	else
		source ./env/bin/activate
	fi
	pip install ../ -U --no-cache-dir
	yes | pip uninstall pymavlink
	pip install "pymavlink"

	python eepromgen.py
	cp $STARTDIR/build/test/eeprom.bin $STARTDIR/build/out/default_eeprom.bin
	python eepromtest.py
) || exit 1;

TARGET_ARCHIVE="sitl-$OSID-$TARGET_LABEL-$TARGET_VERSION.tar.gz"

cd $STARTDIR
mkdir -p $STARTDIR/publish/$TARGET_LABEL
tar -cvf $STARTDIR/publish/$TARGET_LABEL/$TARGET_ARCHIVE -C $STARTDIR/build/out .
