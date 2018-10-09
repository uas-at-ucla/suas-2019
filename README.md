# UAS@UCLA 2018 Drone Code
Code used on our drone competing in the AUVSI SUAS competition.

| CI Tool | Status |
| ---      | ---       |
| Jenkins | [![Jenkins Build Status](https://uasatucla.org/jenkins/buildStatus/icon?job=drone_code-master)](https://uasatucla.org/jenkins/blue/organizations/jenkins/drone_code-master/activity) |
| Travis.CI | [![Travis.CI Build Status](https://travis-ci.com/uas-at-ucla/drone_code.svg?token=vswHzoLKgsSxcZysVnEN&branch=master)](https://travis-ci.com/uas-at-ucla/drone_code) |

![FlightDeck Ground Interface](https://i.imgur.com/n9vinQs.jpg)

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

[Travis-CI](https://travis-ci.org/uas-at-ucla/drone_code) is used to
automatically build every commit that is pushed to this Github repository.

### Installation
Install docker, tmux, and python2.7. After that, "./uas.sh" will list all of the
possible commands that can be run. Use "./uas.sh build" to build the code.

### Platforms and Libraries Used
 * [PX4 Firmware](https://github.com/PX4/Firmware) as the flight controller software stack and simulator.
 * [Docker](https://github.com/docker/docker-ce) for containment of the development environment.
 * [OpenCV](https://github.com/opencv/opencv) for image filtering and segmentation
 * [Darkflow](https://github.com/thtrieu/darkflow) for image classification
 * [MAVLink Common Messages](http://mavlink.org/messages/common) for communicating between Ground Control Station and the UAV
 * [Bazel](https://bazel.build/), a C++ cross-compiler for the Raspberry Pi
 * [Gazebo](https://dev.px4.io/en/simulation/gazebo.html) for 3D simulation
