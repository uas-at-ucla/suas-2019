#!/bin/sh

# Run server and React
# todo: React should be syncing automatically during front-end development - needs to be resolved - must fix webpack
# todo: move all 'www' content into 'client'
# todo: Need to get map cached offline on react
# todo: this needs to be updated when most python files are moved into server directory

cd client && npm run build && cd .. && python run_ground.py
