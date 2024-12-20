#!/bin/bash
set -e # Exit if any program returns an error.

#################################################################
# Make the host C++ project.
#################################################################
if [ ! -d debug ]; then
    mkdir debug
fi
cd debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make

#################################################################
# Make the guest Flutter project.
#################################################################
if [ ! -d myapp ]; then
    flutter create myapp
    cd myapp
    flutter pub add flutter_spinkit
    cd ..
fi
cd myapp
cp ../../main.dart lib/main.dart
flutter build bundle \
        --local-engine-src-path ../../../../../ \
        --local-engine=host_debug_unopt \
        --local-engine-host=host_debug_unopt
cd -

#################################################################
# Run the Flutter Engine Embedder
#################################################################
./flutter_glfw ./myapp ../../../third_party/icu/common/icudtl.dat
