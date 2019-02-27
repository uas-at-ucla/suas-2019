#!/bin/bash

CC_FILES=$(find src/controls -name \*.h -print -o -name \*.cpp -o -name \*.cc -print)
CC_FILES="$CC_FILES $(find lib -name \*.h -print -o -name \*.cpp -o -name \*.cc -print)"
PASSED=0
FAILED=0

for FILE in $CC_FILES
do
  cat $FILE | clang-format-7 -style=file -output-replacements-xml | grep -c "<replacement " > /dev/null

  if [[ $? == 0 ]]
  then
    printf "\033[91mLINT FAILED\033[0m for $FILE\n"
    FAILED=$(expr $FAILED + 1)
  else
    printf "\033[92mLINT PASSED\033[0m for $FILE\n"
    PASSED=$(expr $PASSED + 1)
  fi
done

TOTAL=$(expr $PASSED + $FAILED)

if [[ $FAILED > 0 ]]
then
  TOTAL=$(expr $PASSED + $FAILED)
  printf "\033[91m$FAILED/$TOTAL files failed the lint test.\033[0m\n"
  exit 1
else
  printf "\033[92mAll $TOTAL files passed the linter!\033[0m\n"
fi

