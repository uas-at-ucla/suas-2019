#!/bin/sh

# Build React app if needed, Run server and React

python "client/build.py" &&
	python "run_ground.py"