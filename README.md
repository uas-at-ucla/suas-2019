# UCLA UAS 2018: Drone Code

Code used on our drone competing in the AUVSI SUAS competition.

### What is contained in the repository:
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


### Stuff used to make this:

 * [Arducopter](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter) as the flight controller platform
 * [Dronekit](https://github.com/dronekit/dronekit-python) for interfacing with the flight controller over serial
 * [Dronekit SITL](https://github.com/dronekit/dronekit-sitl) for flight controller simulation and testing
 * [Open CV](https://github.com/opencv/opencv) for image filtering and segmentation
 * [Tensorflow](https://github.com/tensorflow/tensorflow) for image classification

