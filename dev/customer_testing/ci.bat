@ECHO off
REM Copyright 2014 The Flutter Authors. All rights reserved.
REM Use of this source code is governed by a BSD-style license that can be
REM found in the LICENSE file.

REM This should match the ci.sh file in this directory.

REM This is called from the LUCI recipes:
REM https://github.com/flutter/flutter/blob/main/dev/bots/suite_runners/run_customer_testing_tests.dart

REM This script does not assume that "flutter update-packages" has been
REM run, to allow CIs to save time by skipping that steps since it's
REM largely not needed to run the flutter/tests tests.
REM
REM However, we do need to update this directory.
SETLOCAL
cd /d %~dp0
ECHO.
ECHO Updating pub packages...
CALL dart pub get

REM Run the cross-platform script.
CALL ..\..\bin\dart.bat ci.dart
