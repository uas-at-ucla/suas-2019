# UAS at UCLA 2018 Drone Code
Code used on our drone competing in the AUVSI SUAS competition.

[![Build Status](https://travis-ci.org/uas-at-ucla/suas_2018.svg?branch=master)](https://travis-ci.org/uas-at-ucla/suas_2018)
[![Quality Gate](https://sonarcloud.io/api/badges/gate?key=suas_2018)](https://sonarcloud.io/dashboard?id=suas_2018)
[![Number of lines](https://sonarcloud.io/api/badges/measure?key=suas_2018&metric=lines)](https://sonarcloud.io/dashboard?id=suas_2018)

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

[Travis-CI](https://travis-ci.org/uas-at-ucla/suas_2018) is used to
automatically build every commit that is pushed to this Github repository.
Results from these builds can be found here:

[Sonarqube](https://sonarcloud.io/dashboard?id=suas_2018) is also used to check
the quality of the code in this repository and find potential bugs and "code
smells" that may cause trouble down the line. This tool is designed to make our
code maintainable in the long run by giving automated feedback on all the
changes that we make.

### Installation
Please see the [Setup documentation](https://github.com/uas-at-ucla/suas_2018/blob/master/SETUP.md).

### Platforms and Libraries Used
 * [Arducopter](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter) as the flight controller platform
 * [Dronekit](https://github.com/dronekit/dronekit-python) for interfacing with the flight controller over serial
 * [Dronekit SITL](https://github.com/dronekit/dronekit-sitl) for flight controller simulation and testing
 * [OpenCV](https://github.com/opencv/opencv) for image filtering and segmentation
 * [Tensorflow](https://github.com/tensorflow/tensorflow) for image classification

:)
