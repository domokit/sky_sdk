// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

import '../../../widgets/spinning_mixed.dart' as demo;

void main() {
  test('layers smoketest for widgets/spinning_mixed.dart', () {
    FlutterError.onError = (FlutterErrorDetails details) {
      throw details.exception;
    };
    demo.main();
  });
}
