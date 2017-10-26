# Ground Station
The Ground Control Station is the most vital link to interface between the drone
and the user. The end goal of this ground control software is to have an
intuitive web application, in which the user click on points in the map and the
drone will take that flight route.

# Content
* Installation
* Framework
* Competition Rules
* Other Teams Ground Control Software
* Extra Objectives
* Timeline
* Back-up Plan
* Platforms and Libraries

## Installation
Please see the [Setup documentation]().

## Framework
* Hardware: Communications system and firmware
  integration.
  * Must use the flight controller's API's and load its libraries. See source
    code for other ground control applications. See MavProxy.
* Back-end: [Interoperability
  System](http://auvsi-suas-competition-interoperability-system.readthedocs.io/en/latest/),
  Flask Framework
* Front-end: React.js
  * React.js is the easiest framework to use for single-page
    applications. Google Chrome will already serve as our GUI.

## Competition Rules
* [Competition rules]() pertaining to the ground station. Read the following pages:
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

## Extra Objectives
* For cyber security we can include mandatory account login if we are making our ground station through the web - it will ask for username and password.

## Timeline
* Quarter 1: Research, construct a solid plan on firmware integration with the software, have a prototyping Flask and React.js framework ready.
  * Goal: Have a working ground control software that communicates with the plane and can be tested.
* Quarter 2: Code, code, and code.
  * Goal: Have a complete ground control software by end of February.
* Quarter 3: Testing, debugging, and adding extra features.
  * Goal: Software should be able to complete all the objectives laid out by the competition rules.

## Back-up Plan
* Use an open source Linux ground control software.

## Platforms and Libraries
See [other open source ground control
software](http://ardupilot.org/copter/docs/common-choosing-a-ground-station.html)
for inspiration.