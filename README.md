# UCLA UAS 2018 Source Code
Code used on our drone competing in the AUVSI SUAS competition.

[![Build Status](https://travis-ci.org/uas-at-ucla/suas_2018.svg?branch=master)](https://travis-ci.org/uas-at-ucla/suas_2018)

### Contents
 * Control code
    * Mission planning & surveying
    * Collision avoidance
    * Flight simulation
    * Failsafe
 * Vision code
    * Camera interface
    * Target identification
    * Shape/letter classification
    * Determining GPS location of targets
    * Gimbal control
    * Synthetic image generation (for testing)
 * Ground station
    * Antenna tracker
    * Telemetry multiplexer (from Wi-Fi and serial interface)
 * Build and deployment scripts to drone

### Continuous Integration and Tests
This project uses continuous integration to avoid breaking old code as new
features are introduced. Read more about the best practices of CI here:
 * [Continuous integration best practices](https://en.wikipedia.org/wiki/Continuous_integration#Best_practices)

These guidelines are also meant to make our code more portable and easier for
new developers to install. In addition, it allows for tests to be automatically
run on any new code that is checked into this repository, which evaluates
everything from target identification accuracy for the vision system to safety
and reliability for the control software.

Travis-CI is used to automatically build every commit that is pushed to this
Github repository. Results from these builds can be found here:
 * [Travis-CI for UAS at UCLA](https://travis-ci.org/uas-at-ucla/suas_2018)

### Installation
Install all of the packages that you will need.
```bash
sudo apt-get update
sudo apt-get install git python2.7 python-pip python-dev build-essential 
```

Download this repository.
```bash
cd ~/Documents
git clone https://github.com/uas-at-ucla/suas_2018.git
cd suas_2018
```

Install Python dependencies.
```bash
pip install -r build/install/requirements.txt
```

### Platforms and Libraries Used
 * [Arducopter](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter) as the flight controller platform
 * [Dronekit](https://github.com/dronekit/dronekit-python) for interfacing with the flight controller over serial
 * [Dronekit SITL](https://github.com/dronekit/dronekit-sitl) for flight controller simulation and testing
 * [Open CV](https://github.com/opencv/opencv) for image filtering and segmentation
 * [Tensorflow](https://github.com/tensorflow/tensorflow) for image classification
