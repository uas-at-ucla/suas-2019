#!/bin/bash

# Go to repository root.
cd "$(dirname "$0")"
cd ../../..

./tools/scripts/controls/exec.sh ./tools/scripts/lint/check_cc_file.sh
