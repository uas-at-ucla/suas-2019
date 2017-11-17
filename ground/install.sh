#!/bin/sh

# Update interop git submodule
# Update node_modules

# todo: should automatically install docker for us
# Future Expansion: Get all python flask installations automatically setup here

git submodule init && git submodule update && cd client && npm install && cd ..