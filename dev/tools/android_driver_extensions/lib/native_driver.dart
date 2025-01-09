// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;
import 'dart:typed_data';

import 'package:flutter_driver/flutter_driver.dart';

import 'package:matcher/matcher.dart';

// Similar to `flutter_test`, we ignore the implementation import.
// ignore: implementation_imports
import 'package:matcher/src/expect/async_matcher.dart';

import 'package:path/path.dart' as path;
import 'package:test_api/test_api.dart';

import 'src/common.dart';

export 'src/backend/android.dart' show AndroidDeviceTarget, AndroidNativeDriver;
export 'src/common.dart' show ByNativeAccessibilityLabel, ByNativeIntegerId, NativeFinder;

part 'src/driver.dart';
part 'src/goldens.dart';
