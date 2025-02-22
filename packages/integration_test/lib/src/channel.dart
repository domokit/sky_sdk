// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';

/// The method channel used to report the result of the tests to the platform.
/// On Android, this is relevant when running instrumented tests.
// CAMILLE: where the method channel to communicate with Java code is defined
const MethodChannel integrationTestChannel = MethodChannel('plugins.flutter.io/integration_test');
