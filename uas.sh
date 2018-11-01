#!/bin/sh

cd "$(dirname "$0")";
python2.7 -u ./tools/scripts/uas.py $@
