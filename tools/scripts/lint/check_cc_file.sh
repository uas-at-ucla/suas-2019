#!/bin/bash

cat $1 | clang-format-3.9 -style=file -output-replacements-xml | grep -c "<replacement " > /dev/null

exit $(expr $? == 0)
