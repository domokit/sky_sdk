// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['web'])
library;

import 'dart:js_interop';
import 'dart:js_interop_unsafe';

import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

@JS('window')
external JSObject get _window;

void main() {
  IntegrationTestWidgetsFlutterBinding();

  test(
    'IntegrationTestWidgetsFlutterBinding on the web should register certain global properties',
    () {
      expect(_window.hasProperty(r'$flutterDriver'.toJS).toDart, true);
      expect(_window.getProperty(r'$flutterDriver'.toJS), isNotNull);

      expect(_window.hasProperty(r'$flutterDriverResult'.toJS).toDart, true);
      expect(_window.getProperty(r'$flutterDriverResult'.toJS), isNull);
    },
  );
}
