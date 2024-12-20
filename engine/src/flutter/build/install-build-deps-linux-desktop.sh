#!/bin/bash
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# Installs the dependencies necessary to build the Linux Flutter shell, beyond
# those installed by install-build-deps.sh.

set -e

sudo apt -y install libfreetype6-dev \
                    libgl-dev        \
                    libx11-dev       \
                    libxcursor-dev   \
                    libxinerama-dev  \
                    libxrandr-dev    \
                    libxxf86vm-dev
