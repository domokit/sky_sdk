#!/usr/bin/env bash
# Copyright 2014 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# This should match the ci.bat file in this directory.

# This is called from the LUCI recipes:
# https://github.com/flutter/flutter/blob/main/dev/bots/suite_runners/run_customer_testing_tests.dart

set -ex

# This script does not assume that "flutter update-packages" has been
# run, to allow CIs to save time by skipping that steps since it's
# largely not needed to run the flutter/tests tests.
#
# However, we do need to update this directory and the tools directory.
dart pub get
(cd ../tools; dart pub get) # used for find_commit.dart below

# Next we need to update the flutter/tests checkout.
#
# We use find_commit.dart so that we pull the version of flutter/tests
# that was contemporary when the branch we are on was created. That
# way, we can still run the tests on long-lived branches without being
# affected by breaking changes on trunk causing changes to the tests
# that wouldn't work on the long-lived branch.
#
# (This also prevents trunk from suddenly failing when tests are
# revved on flutter/tests -- if you rerun a passing customer_tests
# shard, it should still pass, even if we rolled one of the tests.)
rm -rf ../../bin/cache/pkg/tests
git clone https://github.com/flutter/tests.git ../../bin/cache/pkg/tests
git -C ../../bin/cache/pkg/tests checkout `dart --enable-asserts ../tools/bin/find_commit.dart . master ../../bin/cache/pkg/tests main`

# Finally, run the tests.
dart --enable-asserts run_tests.dart --skip-on-fetch-failure --skip-template ../../bin/cache/pkg/tests/registry/*.test
