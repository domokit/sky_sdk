#!/bin/bash
set -e # Exit if any program returns an error.

if uname -m | grep "arm64"; then
    variant="host_debug_unopt_arm64"
else
    variant="host_debug_unopt"
fi

#################################################################
# Make the host C++ project.
#################################################################
if [ ! -d debug ]; then
    mkdir debug
fi
cd debug
cmake -DCMAKE_BUILD_TYPE=Debug -DFLUTTER_ENGINE_VARIANT=$variant ..
make

#################################################################
# Make the guest Flutter project.
#################################################################
if [ ! -d myapp ]; then
    flutter create myapp
fi

cd myapp
flutter pub add flutter_gpu --sdk=flutter
cp ../../main.dart lib/main.dart
flutter build bundle \
        --local-engine-src-path ../../../../../ \
        --local-engine=$variant \
        --local-engine-host=$variant
cd -

#################################################################
# Run the Flutter Engine Embedder
#################################################################
./flutter_glfw ./myapp ../../../third_party/icu/common/icudtl.dat
