#!/bin/bash

CC_FILES=$(find src -name \*.h -print -o -name \*.cpp -o -name \*.cc -print)
CC_FILES="$CC_FILES $(find lib -name \*.h -print -o -name \*.cpp -o -name \*.cc -print)"
PASSED=0
FAILED=0

for FILE in $CC_FILES
do
  echo "Formatting $FILE"
  clang-format-7 -style=file -i $FILE
done
