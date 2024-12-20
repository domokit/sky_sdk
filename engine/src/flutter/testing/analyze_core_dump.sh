#!/bin/bash

# Gather information from a core dump.
#
# This script can be invoked by the run_tests.py script after an
# engine test crashes.

BUILDROOT=$1
EXE=$2
CORE=$3
OUTPUT=$4

UNAME=$(uname)
if [ "$UNAME" == "Linux" ]; then
  if [ -x "$(command -v gdb)" ]; then
    GDB=gdb
  else
    GDB=$BUILDROOT/flutter/third_party/android_tools/ndk/prebuilt/linux-x86_64/bin/gdb
  fi
  echo "GDB=$GDB"
  $GDB $EXE $CORE --batch -ex "thread apply all bt" > $OUTPUT
fi
