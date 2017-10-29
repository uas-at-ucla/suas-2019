#!/bin/bash
# Configures the Interop Client for the Docker image build.

CLIENT=$(readlink -f $(dirname ${BASH_SOURCE[0]}))

export PYTHONPATH=$CLIENT
source ${CLIENT}/venv2/bin/activate
