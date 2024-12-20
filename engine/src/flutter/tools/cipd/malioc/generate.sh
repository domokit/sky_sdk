#!/bin/bash
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

set -e

version_tag=`date +%Y-%m-%dT%T%z`

cipd create --pkg-def cipd.yaml -tag last_updated:"$version_tag"
