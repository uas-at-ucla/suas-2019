#!/bin/bash

cd "$(dirname "$0")";

autossh -M 20000 -N -R 9091:0.0.0.0:22 uas@uasatucla.org
