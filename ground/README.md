# Ground Station
The Ground Control Station is the most vital link to interface between the drone
and the user. The end goal of this ground control software is to have an
intuitive web application, in which the user click on points in the map and the
drone will take that flight route.

# Content
* Installation
* Compilation
* Framework
* Competition Rules
* Other Teams Ground Control Software
* Platforms and Libraries

## Installation
   Make sure you have all of the Python dependencies:
   ```bash
   pip install -r ../build/pip_requirements.txt
   ```

   Install Node from [nodejs.org](https://nodejs.org)

   Install Node Modules:
   ```bash
   chmod +x install.sh
   ./install.sh
   ```

## Compilation
   To make a static build and launch it:
   ```bash
   chmod +x launch.sh
   ./launch.sh
   ```

   To build without launching:
   ```bash
   cd client
   npm run build
   ```

   To launch an existing build:
   ```
   python server/run_ground.py
   ```

   To develop with hot reloading:
   ```bash
   cd client
   npm start
   ```
   Then in another shell:
   ```bash
   python sever/run_backend.py
   ```

## Framework
* Hardware: Communications system and firmware
  integration.
  * Must use the flight controller's API's and load its libraries. See source
    code for other ground control applications. See MavProxy.
* Back-end: [Interoperability
  System](http://auvsi-suas-competition-interoperability-system.readthedocs.io/en/latest/),
  Flask Framework, MySQL (future expansion)
* Front-end: React.js

## Competition Rules
* [Competition rules](https://github.com/uas-at-ucla/suas_2018/blob/master/ground/pdfs/comp_rules.pdf) pertaining to the ground station. Read the following pages:
  * 10-11
  * 13
  * 16-17 (on obstacle avoidance section if we are including this in ground station)
  * 21
  * 28-30

## Other Teams Ground Control Software
* Look at the ground control software that other teams have used in the past through the [AUVSI SUAS website](http://www.auvsi-suas.org/competitions/2017/).

* These are the teams that built their own custom ground control software:
  * 2nd Place: [Cornell's custom ground control software](http://www.auvsi-suas.org/static/competitions/2017/journals/auvsi_suas-2017-journals-cornell_university.pdf) is on page 7.
  * 10th Place: [BYU's custom ground control software](http://www.auvsi-suas.org/static/competitions/2017/journals/auvsi_suas-2017-journals-cornell_university.pdf) is on page 8.
  * 11th Place: [Alberta's custom ground control software](http://www.auvsi-suas.org/static/competitions/2017/journals/auvsi_suas-2017-journals-university_of_alberta.pdf). See pages 9, 10, 16.
  * 15th Place: [A school from Karnataka, India](http://www.auvsi-suas.org/static/competitions/2017/journals/auvsi_suas-2017-journals-ms_ramaiah.pdf). See pages 12, 13-14, 17.
  * All other schools mainly uesd either QGroundControl or Mission Planner.

## Platforms and Libraries
See [other open source ground control
software](http://ardupilot.org/copter/docs/common-choosing-a-ground-station.html)
for inspiration.
