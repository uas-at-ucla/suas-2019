#!/bin/bash
# Installs software for tools.

TOOLS=$(readlink -f $(dirname ${BASH_SOURCE[0]}))
LOG_NAME=setup_tools
source ${TOOLS}/common.sh

log "Installing APT packages."
apt-get -qq update
apt-get -qq install -y \
    graphviz \
    postgresql-client \
    protobuf-compiler \
    python-matplotlib \
    python-nose \
    python-numpy \
    python-pip \
    python-psycopg2 \
    python-pyproj \
    python-scipy \
    python-virtualenv

log "Building tools virtualenv."
(cd ${TOOLS} && \
    virtualenv -p /usr/bin/python2 ${TOOLS}/venv && \
    source ${TOOLS}/venv/bin/activate && \
    pip install -U -r requirements.txt && \
    deactivate)
