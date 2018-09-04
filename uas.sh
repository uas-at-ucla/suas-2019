#!/bin/sh

cd "$(dirname "$0")";
python2.7 ./tools/scripts/uas.py $@
